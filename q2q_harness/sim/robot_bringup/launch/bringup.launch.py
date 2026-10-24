"""统一启动 Gazebo Fortress + Husky/Kinova + 控制器 + 可选导航。

    ros2 launch robot_bringup bringup.launch.py gui:=false world:=q2q_warehouse nav:=true

robot.yaml 经官方生成器展开，再由 model_patches 修正轮胎接触并添加物理真值 odom。
命名空间 a200_0000；frame 链 map→odom→base_footprint→base_link→机械臂/激光。
TF 话题在 /a200_0000/tf、/a200_0000/tf_static；/clock 在全局。
nav:=true 会等四个控制器 active 后启动 Nav2，失败时中止，不靠固定等待猜状态。
"""

import os
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PKG = 'robot_bringup'
CONFIG_NAME = 'bringup.yaml'
GZ_VERSION = '6'  # Fortress（ros_gz 0.244 对应 gz-sim6；VM 里另有 Harmonic 8.15，此处不用）
CLOCK_BRIDGE_ARG = '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'

# 控制器自检的延迟：必须晚于 Clearpath 那批 spawner（它们在生成的 10 s 内跑完），
# 又要早于 Nav2 的 30 s —— 否则 Nav2 起来时 map 帧还是不存在。
CONTROLLER_CHECK_DELAY = 20.0
CONTROLLER_CHECK_TIMEOUT = 120.0
VIRTUAL_LIDAR_DELAY = 12.0   # 等 RSP 把 TF 发出来，虚拟激光才能查到激光位姿

# 参数默认值 —— 与 config/bringup.yaml 的键名一一对应
DEFAULTS = {
    'world': 'q2q_empty',
    'gui': True,
    'rviz': False,
    'use_sim_time': True,
    'generate': True,
    'gz_verbose': 4,
    'lidar_update_rate': 15.0,
    'x': 0.0,
    'y': 0.0,
    'z': 0.15,
    'yaw': 0.0,
    'setup_path': '~/.q2q/setup/a200_kinova',
    'nav': False,
    'slam': False,
}


def _default_config_path():
    return os.path.join(get_package_share_directory(PKG), 'config', CONFIG_NAME)


def _load_config(path):
    try:
        with open(path, 'r', encoding='utf-8') as handle:
            loaded = yaml.safe_load(handle) or {}
    except FileNotFoundError:
        return {}
    return loaded if isinstance(loaded, dict) else {}


def _as_bool(value):
    return str(value).strip().lower() in ('true', '1', 'yes')


def _as_str(value):
    """YAML 里的 True/0.15/4 统一成 launch 认的字符串。"""
    if isinstance(value, bool):
        return 'true' if value else 'false'
    return str(value)


def _world_file(value):
    """world 参数 -> 绝对 .sdf 路径。解析顺序：绝对路径 -> 本包世界 -> Clearpath 世界。"""
    raw = str(value).strip()
    if raw.endswith('.sdf'):
        raw = raw[:-4]
    if os.path.isabs(raw):
        return raw + '.sdf'
    for share_pkg in ('robot_worlds', 'clearpath_gz'):
        try:
            candidate = os.path.join(get_package_share_directory(share_pkg), 'worlds', raw + '.sdf')
        except Exception:  # 包没装也要能启动（交给 Gazebo 自己找）
            continue
        if os.path.isfile(candidate):
            return candidate
    return raw + '.sdf'


def _world_name(value):
    """world 参数 -> world name。必须和 .sdf 里的 <world name="..."> 一致，否则传感器话题前缀对不上。"""
    path = _world_file(value)
    world = ET.parse(path).getroot().find('world')
    if world is None or not world.get('name'):
        raise ValueError('SDF must define a named world: '+path)
    return world.get('name')


def _sync_robot_yaml(setup_path):
    """包内 config/robot.yaml 是唯一源；与 setup 树里的不一致就覆盖。"""
    src = os.path.join(get_package_share_directory(PKG), 'config', 'robot.yaml')
    dst = os.path.join(setup_path, 'robot.yaml')
    os.makedirs(setup_path, exist_ok=True)
    if os.path.isfile(dst):
        with open(src, 'rb') as new, open(dst, 'rb') as old:
            if new.read() == old.read():
                return dst
    shutil.copyfile(src, dst)
    return dst


def _namespace_of(robot_yaml):
    with open(robot_yaml, 'r', encoding='utf-8') as handle:
        data = yaml.safe_load(handle) or {}
    system = data.get('system') or {}
    ros2 = system.get('ros2') or {}
    return str(ros2.get('namespace') or '')


def _write_gui_config(setup_path, namespace):
    """复制 Clearpath 的 Gazebo GUI 配置，把 Teleop 面板指到命名空间下的 cmd_vel。"""
    src = os.path.join(get_package_share_directory('clearpath_gz'), 'config', 'gui.config')
    dst = os.path.join(setup_path, 'gui.config')
    with open(src, 'r', encoding='utf-8') as handle:
        content = handle.read()
    topic = '/cmd_vel' if namespace in ('', '/') else '/{0}/cmd_vel'.format(namespace)
    content = content.replace(
        '<plugin filename="Teleop">',
        '<plugin filename="Teleop">\n      <topic>{0}</topic>'.format(topic))
    with open(dst, 'w', encoding='utf-8') as handle:
        handle.write(content)
    return dst


def _resource_paths():
    """Gazebo 找模型/世界的资源路径：Clearpath 世界 + 本包世界 + 所有已 source 的包 share。"""
    paths = []
    for share_pkg in ('clearpath_gz', 'robot_worlds'):
        paths.append(os.path.join(get_package_share_directory(share_pkg), 'worlds'))
    for prefix in os.environ.get('AMENT_PREFIX_PATH', '').split(':'):
        if not prefix:
            continue
        share = os.path.join(prefix, 'share')
        if share not in paths:
            paths.append(share)
    return ':'.join(paths)


def _prepare_model(setup_path, generate, lidar_rate):
    """生成 -> 打补丁 -> 校验（调 scripts/prepare_setup.py）。

    这一步失败必须让 launch 中止：补丁类改动最危险的失败方式是「没打上但照常跑」，
    所以这里用 RuntimeError 掐断，不进入启动阶段。
    """
    script = os.path.join(get_package_share_directory(PKG), 'scripts', 'prepare_setup.py')
    command = [sys.executable, script, '--setup', setup_path,
               '--lidar-rate', _as_str(lidar_rate)]
    if not generate:
        command.append('--skip-generate')
    done = subprocess.run(command, capture_output=True, text=True)
    for line in (done.stdout or '').splitlines():
        print(line)
    if done.returncode != 0:
        print(done.stderr or '')
        raise RuntimeError('模型准备失败，已中止启动（看上面的 [prepare] / [patches] 输出）')


def _launch_setup(context, *args, **kwargs):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    world_arg = arg('world')
    gui = _as_bool(arg('gui'))
    setup_path = os.path.expanduser(arg('setup_path'))

    world_file = _world_file(world_arg)
    world_name = _world_name(world_arg)
    robot_yaml = _sync_robot_yaml(setup_path)
    namespace = _namespace_of(robot_yaml)
    _prepare_model(setup_path, _as_bool(arg('generate')), arg('lidar_update_rate'))

    gz_args = '{0} -r -v {1}'.format(world_file, arg('gz_verbose'))
    if gui:
        gz_args += ' --gui-config ' + _write_gui_config(setup_path, namespace)
        # ogre2（默认）在这台 SVGA3D 上 3D 视口是空白的（只有工具栏，日志报 Ogre2RayQuery
        # non-finite origin）；换 Ogre 1.x 能正常渲染——实测 2026-10-24 21:04，同一天 RViz
        # 也会用 Ogre 1.9 跑出 31 fps，说明不是 VM 没有 GL，是 ogre2 这条路径不行。
        gz_args += ' --render-engine ogre'
    else:
        # `-s` = 只起 server。默认再加 --headless-rendering（ogre2 走 EGL，不依赖窗口系统）；
        # 设 Q2Q_GLX=1 则改用 X 上的 GLX（配合 DISPLAY + LIBGL_ALWAYS_SOFTWARE 走 llvmpipe）。
        # 两条路都试过：本机 SVGA3D 只报 GL 4.1 且 Accelerated: no，GPU 激光拿不到深度。
        if os.environ.get('Q2Q_GLX') == '1':
            gz_args += ' -s'
        else:
            gz_args += ' -s --headless-rendering'

    resource_env = _resource_paths()
    actions = [
        SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', resource_env),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', resource_env),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
            launch_arguments=[('gz_args', gz_args), ('gz_version', GZ_VERSION)],
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='clock_bridge',
            output='screen',
            arguments=[CLOCK_BRIDGE_ARG],
        ),
        Node(
            package='ros_gz_bridge', executable='parameter_bridge',
            name='ground_truth_bridge', output='screen',
            parameters=[{'use_sim_time': _as_bool(arg('use_sim_time'))}],
            arguments=[
                '/model/{0}/robot/ground_truth_odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry'.format(namespace),
                '/model/{0}/robot/ground_truth_tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V'.format(namespace)],
            remappings=[
                ('/model/{0}/robot/ground_truth_odom'.format(namespace), '/{0}/ground_truth/odom'.format(namespace)),
                ('/model/{0}/robot/ground_truth_tf'.format(namespace), '/{0}/tf'.format(namespace))],
        ),
        # 机器人生成 + spawn + 底盘/传感器/机械臂控制器 + 可选 RViz 全在 Clearpath 这个 launch 里
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('clearpath_gz'), 'launch', 'robot_spawn.launch.py')),
            launch_arguments=[
                ('setup_path', setup_path),
                ('world', world_name),
                ('use_sim_time', arg('use_sim_time')),
                ('rviz', arg('rviz')),
                # 生成/补丁已经由 _prepare_model 做完，这里绝不能再让 Clearpath 重新生成一次
                ('generate', 'false'),
                ('x', arg('x')),
                ('y', arg('y')),
                ('z', arg('z')),
                ('yaw', arg('yaw')),
            ],
        ),
    ]

    # 虚拟激光（几何射线）：这台 VM 的 GPU 渲染拿不到深度，gpu_lidar 的 720 束里
    # 只有 8 束有值且全 = 24.999，slam 扫不出任何障碍。scripts/virtual_lidar.py
    # 读世界 SDF 的长方体做 2D 射线求交，发标准 LaserScan；原桥接已改名到 scan_raw。
    actions.append(TimerAction(period=VIRTUAL_LIDAR_DELAY, actions=[
        ExecuteProcess(
            cmd=[sys.executable,
                 os.path.join(get_package_share_directory(PKG), 'scripts', 'virtual_lidar.py'),
                 '--world', world_file,
                 '--topic', '/{0}/sensors/lidar2d_0/scan'.format(namespace),
                 # 不加这两条重映射，tf2_ros 订的是绝对名 /tf，而 Clearpath 把 TF 发在
                 # /a200_0000/tf —— 查不到激光位姿，就一帧都不发（2026-10-21 实测）
                 '--ros-args',
                 # use_sim_time 必须开：否则扫描时间戳是墙钟（1.79e9），
                 # 而 TF 用仿真时间，slam 的 MessageFilter 永远查不到对应时刻的变换，
                 # 现象是 "frame 'lidar2d_0_laser' at time 1792591695.512 ... queue is full"
                 '-p', 'use_sim_time:=true',
                 '-r', '/tf:=/{0}/tf'.format(namespace),
                 '-r', '/tf_static:=/{0}/tf_static'.format(namespace)],
            output='screen',
        ),
    ]))

    # 控制器自检（第 3 步加）：Clearpath 的 spawner 在「CM 被渲染初始化堵住」时会
    # load_controller 超时 10 s，重试撞上 already loaded 就 FATAL 退出，控制器停在 unconfigured
    # ——现象是 /joint_states 没有、map 帧不存在、slam/Nav2 全线起不来（2026-10-21 实测）。
    # spawner 不幂等，所以这里跑一个只补缺的脚本，让启动对时序不敏感。
    controller_check = ExecuteProcess(
            cmd=[sys.executable,
                 os.path.join(get_package_share_directory(PKG), 'scripts', 'ensure_controllers.py'),
                 '--namespace', namespace],
            output='screen',
        )
    actions.append(TimerAction(period=CONTROLLER_CHECK_DELAY, actions=[controller_check]))

    def start_gripper(event, context):
        if event.returncode != 0:
            return []
        return [ExecuteProcess(cmd=[sys.executable,
            os.path.join(get_package_share_directory(PKG), 'scripts', 'gripper_action.py'),
            '--ros-args', '-p', 'use_sim_time:=true'], output='screen')]
    actions.append(RegisterEventHandler(OnProcessExit(
        target_action=controller_check, on_exit=start_gripper)))

    # nav:=true 才起 Nav2（第 3 步）。默认关，保证第 1/2 步的启动行为一个字都不变。
    #
    # 以控制器状态作为启动门槛；固定等待 30 秒不能证明机器人准备好了。
    if _as_bool(arg('nav')):
        def start_navigation(event, context):
            if event.returncode != 0:
                raise RuntimeError('控制器验收失败，拒绝启动 Nav2；请查看 ensure_controllers 输出')
            return [IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(
                    get_package_share_directory(PKG), 'launch', 'nav2.launch.py')),
                launch_arguments=[('slam', arg('slam'))],
            )]
        actions.append(RegisterEventHandler(OnProcessExit(
            target_action=controller_check, on_exit=start_navigation)))

    summary = (
        '\n================ robot_bringup ================\n'
        '  world       : {world}   ({world_file})\n'
        '  gui / rviz  : {gui} / {rviz}\n'
        '  namespace   : {ns}\n'
        '  setup_path  : {setup}\n'
        '  generate    : {generate}\n'
        '  TF 根       : base_footprint（高度由当前平台模型决定）\n'
        '  验收        : ros2 node list / ros2 topic list\n'
        '                ros2 topic echo /clock --once\n'
        '=============================================='
    ).format(
        world=world_name,
        world_file=world_file,
        gui=arg('gui'),
        rviz=arg('rviz'),
        ns=namespace or '<none>',
        setup=setup_path,
        generate=arg('generate'),
    )
    actions.insert(0, LogInfo(msg=summary))
    return actions


def generate_launch_description():
    config_default = _default_config_path()
    overrides = _load_config(config_default)
    merged = dict(DEFAULTS)
    for key, value in overrides.items():
        if key in merged:
            merged[key] = value

    def declare(name, description):
        return DeclareLaunchArgument(
            name, default_value=_as_str(merged[name]), description=description)

    arguments = [
        DeclareLaunchArgument(
            'config', default_value=config_default,
            description='参数 YAML（默认 <pkg>/config/bringup.yaml）'),
        declare('world', 'Gazebo 世界名或 .sdf 路径'),
        declare('gui', 'true = 开 Gazebo 界面；false = 只开 server'),
        declare('rviz', 'true = 同时开 RViz'),
        declare('use_sim_time', '仿真时间'),
        declare('generate', 'true = 启动前按 config/robot.yaml 重新生成 setup 树'),
        declare('gz_verbose', 'Gazebo 日志级别 1-4'),
        declare('lidar_update_rate', '激光 update_rate（Hz）；生成器写死 50，这里覆盖'),
        declare('x', '机器人初始 x（m）'),
        declare('y', '机器人初始 y（m）'),
        declare('z', '机器人初始 z（m）'),
        declare('yaw', '机器人初始 yaw（rad）'),
        declare('setup_path', 'Clearpath 生成树目录（可写，不进代码仓）'),
        declare('nav', 'true = 同时起 Nav2（第 3 步，含 slam/AMCL 开关）'),
        declare('slam', 'nav 为 true 时有效：true = 建图（只起 slam_toolbox）'),
    ]
    return LaunchDescription(arguments + [OpaqueFunction(function=_launch_setup)])
