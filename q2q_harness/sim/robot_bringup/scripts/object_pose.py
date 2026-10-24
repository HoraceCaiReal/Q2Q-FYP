#!/usr/bin/env python3
"""object_pose：读 Gazebo 里实体的**真实位姿**（世界帧，含偏航）。

为什么需要：`virtual_camera` 是按世界文件的 `<pose>` 渲染的，看不见被搬动的物体；
判断「物体是否跟手」、以及第 6 步算抓取目标，都必须查仿真里实体的真实位姿。

一次抓取只读一帧，同一帧里能同时拿到底盘、抓取箱与**两指指尖**的真实位置——
指尖用真值而不是 FK，是因为 MoveIt 看不到 mimic，FK 算出来的指尖是推的、不是量的。

用法（VM 内）：
    python3 object_pose.py --model pick_bin
    python3 object_pose.py --model pick_bin --expect 1.20 0.0 0.62
    python3 object_pose.py --models base_footprint,pick_bin,arm_0_gripper_left_finger_dist_link
"""

import argparse
import json
import math
import re
import subprocess

WORLD = 'q2q_warehouse'
TOPIC = '/world/%s/pose/info' % WORLD
NAME = re.compile(r'name: "([^"]+)"')
POSITION = re.compile(r'position \{(.*?)\}', re.S)
ORIENTATION = re.compile(r'orientation \{(.*?)\}', re.S)


def _fields(body):
    """从 protobuf 文本块里取 x/y/z/w —— **缺字段按 0 处理**。

    坑：protobuf 文本格式会省略零值字段。`pick_stand` 的 y=0、`base_footprint` 的 x=y=0，
    于是它们的 position 块里根本没有 `x:`/`y:` 行；用「x、y、z 三个必须都在」的正则会把
    这两个实体整条丢掉（实测踩过：报「实体不在世界里」）。
    orientation 块存在时，缺省 w 也必须按 0（合法的 180° 转动）。整个块缺失才用单位旋转。
    """
    values = {}
    for key in ('x', 'y', 'z', 'w'):
        found = re.search(r'(?:^|\s)%s: ([-+0-9.eE]+)' % key, body)
        values[key] = float(found.group(1)) if found else 0.0
    return values


def parse_entities(output):
    """Parse physical local/model poses; preserve valid w=0 half-turns."""
    stamp = re.search(r'stamp\s*\{(.*?)\}', output, re.S)
    def integer(key):
        found = re.search(r'\b%s:\s*(\d+)' % key, stamp.group(1)) if stamp else None
        return int(found.group(1)) if found else 0
    stamp_ns = integer('sec') * 10**9 + integer('nsec')
    entities = {}
    for block in output.split('pose {')[1:]:
        name, position = NAME.search(block), POSITION.search(block)
        if not name or not position:
            continue
        orientation = ORIENTATION.search(block)
        p = _fields(position.group(1))
        q = _fields(orientation.group(1)) if orientation else dict(x=0., y=0., z=0., w=1.)
        norm = math.sqrt(sum(v*v for v in q.values()))
        if not math.isfinite(norm) or norm < 1e-9:
            raise ValueError('Invalid quaternion for '+name.group(1))
        q = {k: v / norm for k, v in q.items()}
        entities[name.group(1)] = {
            'position': (p['x'], p['y'], p['z']),
            'quaternion': tuple(q[k] for k in ('x', 'y', 'z', 'w')),
            'stamp_ns': stamp_ns,
            'yaw': math.atan2(2*(q['w']*q['z']+q['x']*q['y']),
                              1-2*(q['y']**2+q['z']**2))}
    if not entities:
        raise ValueError('Gazebo pose response contains no entities')
    return entities


def read_entities():
    """抓一帧 pose/info -> {名字: {'position': (x,y,z), 'yaw': rad}}。

    按 'pose {' 切块而不是按花括号配对：position/orientation 自己是多行花括号块，
    用 `\\n}` 收尾会先切在 position 上，导致一半实体解析不出 orientation（实测踩过）。
    """
    output = subprocess.run(['ign', 'topic', '-e', '-t', TOPIC, '-n', '1'],
                            capture_output=True, text=True, check=True, timeout=10).stdout
    return parse_entities(output)


def read_poses():
    """只取位置，保持旧接口（{名字: (x, y, z)}）。"""
    return {name: item['position'] for name, item in read_entities().items()}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--model', default='')
    parser.add_argument('--models', default='', help='逗号分隔，一次取多个实体')
    parser.add_argument('--expect', nargs=3, type=float, default=None)
    args = parser.parse_args()

    entities = read_entities()
    names = [n for n in (args.models.split(',') if args.models else [args.model]) if n]
    missing = [n for n in names if n not in entities]
    if missing:
        print(json.dumps({'error': '实体不在世界里', 'missing': missing,
                          'known': sorted(entities)}, ensure_ascii=False))
        return 1

    summary = {'entities': {n: {'position': [round(v, 4) for v in entities[n]['position']],
                               'yaw_deg': round(math.degrees(entities[n]['yaw']), 2)}
                            for n in names}}
    if args.expect and len(names) == 1:
        delta = [round(entities[names[0]]['position'][i] - args.expect[i], 4) for i in range(3)]
        summary['displacement_from_expect'] = delta
        summary['moved_m'] = round(sum(v * v for v in delta) ** 0.5, 4)
    print(json.dumps(summary, ensure_ascii=False))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
