"""Nav2/SLAM 入口（a200_0000 命名空间）。

默认：几何真值地图 + map→odom 恒等变换 + Gazebo 物理真值 odom + NavFn/RPP。
localization:=amcl 可启用 AMCL，但它不属于当前已验收的真值基线。
slam:=true 只启动 slam_toolbox。TF 话题统一为 /a200_0000/tf 和 tf_static。
"""

import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace, SetParameter
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml

NAMESPACE = 'a200_0000'


def _cap_bool(value):
    """'true'/'false'/任何真值写法 -> 'True'/'False'。原因是 PythonExpression 会真求值。"""
    return 'True' if str(value).strip().lower() in ('true', '1', 'yes') else 'False'


def _slam_actions(context, arg):
    """建图：slam_toolbox 在 a200_0000 命名空间里，参数补上命名空间前缀，TF 强制走相对名。"""
    params = ParameterFile(RewrittenYaml(
        source_file=arg('slam_params_file'),
        root_key=NAMESPACE,          # 顶层键 slam_toolbox -> a200_0000/slam_toolbox
        param_rewrites={'use_sim_time': arg('use_sim_time')},
        convert_types=True), allow_substs=True)
    return [GroupAction([
        PushRosNamespace(NAMESPACE),
        Node(package='slam_toolbox', executable='sync_slam_toolbox_node', name='slam_toolbox',
             parameters=[params], output='screen',
             # slam_toolbox 内部订的是绝对名 /tf，不重映射就看不到 Clearpath 的 TF（实测）
             remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')]),
    ])]


def _navigation_actions(arg):
    if arg('localization') == 'ground_truth':
        share = get_package_share_directory('nav2_bringup')
        return [GroupAction([
            PushRosNamespace(NAMESPACE),
            SetParameter(name='bond_timeout', value=float(arg('bond_timeout'))),
            Node(package='tf2_ros', executable='static_transform_publisher',
                 name='ground_truth_map_to_odom',
                 arguments=['--frame-id', 'map', '--child-frame-id', 'odom'],
                 remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
                 parameters=[{'use_sim_time': _cap_bool(arg('use_sim_time')) == 'True'}]),
            Node(package='nav2_map_server', executable='map_server', name='map_server',
                 parameters=[{'yaml_filename': arg('map'),
                              'use_sim_time': _cap_bool(arg('use_sim_time')) == 'True'}], output='screen'),
            Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
                 name='lifecycle_manager_localization', parameters=[{
                     'use_sim_time': _cap_bool(arg('use_sim_time')) == 'True',
                     'autostart': _cap_bool(arg('autostart')) == 'True',
                     'bond_timeout': float(arg('bond_timeout')), 'node_names': ['map_server']}], output='screen'),
            ExecuteProcess(cmd=[sys.executable, os.path.join(
                get_package_share_directory('robot_bringup'), 'scripts', 'map_startup.py')], output='screen'),
            IncludeLaunchDescription(PythonLaunchDescriptionSource(
                os.path.join(share, 'launch', 'navigation_launch.py')),
                launch_arguments=[('namespace', NAMESPACE),
                                  ('use_sim_time', arg('use_sim_time')),
                                  ('autostart', arg('autostart')),
                                  ('params_file', arg('params_file')),
                                  ('use_composition', 'False')]),
        ])]
    if arg('localization') != 'amcl':
        raise ValueError('localization must be ground_truth or amcl')
    return [GroupAction([SetParameter(name='bond_timeout', value=float(arg('bond_timeout'))), IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')),
        launch_arguments=[
            ('namespace', NAMESPACE),          # 决定 TF / 话题前缀，方案 A 的全部内容
            ('use_namespace', 'True'),         # 让 nav2_bringup 自己 PushRosNamespace
            ('map', arg('map')),
            ('slam', 'False'),                 # 建图那一支由 _slam_actions 负责
            ('use_sim_time', arg('use_sim_time')),
            ('autostart', arg('autostart')),
            ('params_file', arg('params_file')),
            ('use_composition', _cap_bool(arg('use_composition'))),
        ],
    )])]


def _launch_setup(context, *args, **kwargs):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    if _cap_bool(arg('slam')) == 'True':
        return _slam_actions(context, arg)
    return _navigation_actions(arg)


def generate_launch_description():
    share = get_package_share_directory('robot_bringup')
    world_share = get_package_share_directory('robot_worlds')
    arguments = [
        # 建图产物放 robot_worlds/maps/ 下。没建过图时这个文件不存在，map_server 会报错
        # ——这是故意的：先跑 slam:=true 建图，再跑这个。
        DeclareLaunchArgument('map',
                              default_value=os.path.join(world_share, 'maps', 'q2q_warehouse_ground_truth.yaml'),
                              description='map_server 读的 yaml；slam:=true 时忽略'),
        DeclareLaunchArgument('slam', default_value='false',
                              description='true = 只起 slam_toolbox；false = 已有地图 + localization 所选定位'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('localization', default_value='ground_truth',
                              description='ground_truth = 物理真值定位基线；amcl = 粒子滤波（另行评测）'),
        DeclareLaunchArgument('bond_timeout', default_value='30.0',
                              description='仿真宿主暂停的心跳预算（墙钟秒）；保持启用故障检测'),
        DeclareLaunchArgument('params_file',
                              default_value=os.path.join(share, 'config', 'nav2.yaml')),
        DeclareLaunchArgument('slam_params_file',
                              default_value=os.path.join(share, 'config', 'slam_toolbox.yaml')),
        # 纯进程模式：每个 Nav2 节点一个进程，ros2 node list 数得清，出问题只看一个人的日志。
        DeclareLaunchArgument('use_composition', default_value='False'),
    ]
    return LaunchDescription(arguments + [OpaqueFunction(function=_launch_setup)])
