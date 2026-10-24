"""准备好 Clearpath 的 setup 树：生成 -> 打补丁 -> 校验。

由 bringup.launch.py 在启动前调用（阻塞式，约 10 秒）；也可以手动跑，排障很快：

    source /opt/ros/humble/setup.bash
    python3 ~/ros2_ws/install/robot_bringup/share/robot_bringup/scripts/prepare_setup.py \
        --setup ~/.q2q/setup/a200_kinova

任何一步失败都以非 0 退出 —— launch 会因此中止，不启动半成品模型。
"""

import argparse
import os
import subprocess
import sys

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import model_patches  # noqa: E402  (必须在改 sys.path 之后导入)

# 顺序不能换：后面的生成器依赖前面的产物
GENERATORS = [
    ('clearpath_generator_common', 'generate_description'),
    ('clearpath_generator_common', 'generate_semantic_description'),
    ('clearpath_generator_gz', 'generate_launch'),
    ('clearpath_generator_gz', 'generate_param'),
]


def _run(cmd):
    print('[prepare] ' + ' '.join(cmd), flush=True)
    done = subprocess.run(cmd, capture_output=True, text=True)
    if done.returncode != 0:
        sys.stderr.write((done.stdout or '')[-1200:] + (done.stderr or '')[-1200:])
        raise SystemExit('[prepare] 失败：%s' % ' '.join(cmd))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--setup', required=True, help='setup 树目录（含 robot.yaml）')
    parser.add_argument('--skip-generate', action='store_true',
                        help='跳过生成，只打补丁 + 校验（排障用）')
    parser.add_argument('--lidar-rate', type=float, default=15.0,
                        help='激光 update_rate（Hz）；生成器写死 50，这里覆盖，默认 15')
    args = parser.parse_args()

    setup = os.path.expanduser(args.setup)
    robot_yaml = os.path.join(setup, 'robot.yaml')
    if not os.path.isfile(robot_yaml):
        raise SystemExit('[prepare] 找不到 %s' % robot_yaml)

    with open(robot_yaml, 'r', encoding='utf-8') as fh:
        config = yaml.safe_load(fh) or {}
    namespace = str(((config.get('system') or {}).get('ros2') or {}).get('namespace') or '')

    if args.skip_generate:
        print('[prepare] 跳过生成，直接打补丁')
    else:
        for package, executable in GENERATORS:
            _run(['ros2', 'run', package, executable, '-s', setup])

    model_patches.apply_all(setup, namespace, args.lidar_rate)
    model_patches.verify(setup, args.lidar_rate)
    print('[prepare] 模型就绪（namespace=%s，激光 %s Hz）'
          % (namespace or '<none>', args.lidar_rate), flush=True)


if __name__ == '__main__':
    main()
