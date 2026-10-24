"""启动 move_group（第 4 步）：只服务手臂/夹爪规划，不碰导航。

参数**故意写在 launch 里**，不走单独的 YAML：
2026-10-21 的坑是命名空间被重写后参数树对不上节点名，ROS 会静默忽略整份文件。
写在 launch 里，节点拿到的就是这份 dict，没有第二层解释。
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ARM_JOINTS = ['arm_0_joint_1', 'arm_0_joint_2', 'arm_0_joint_3',
              'arm_0_joint_4', 'arm_0_joint_5', 'arm_0_joint_6']

# 顺序与 nav2 一致：先标定，再做时间最优参数化，最后修边界/碰撞
ADAPTERS = ' '.join([
    'default_planner_request_adapters/AddTimeOptimalParameterization',
    'default_planner_request_adapters/ResolveConstraintFrames',
    'default_planner_request_adapters/FixWorkspaceBounds',
    'default_planner_request_adapters/FixStartStateBounds',
    'default_planner_request_adapters/FixStartStateCollision',
    'default_planner_request_adapters/FixStartStatePathConstraints',
])

# 控制器映射：move_group 只认这一个出口，名字与第 4 步实测的 action 一致
CONTROLLERS = {
    'controller_names': ['arm_0_joint_trajectory_controller'],
    'arm_0_joint_trajectory_controller': {
        'type': 'FollowJointTrajectory',
        'action_ns': 'follow_joint_trajectory',
        'joints': ARM_JOINTS,
        'joint_names': ARM_JOINTS,  # 小版本间键名不一致，两份都写，多出的那份不会被读
    },
}


def _text(path):
    return Path(path).read_text(encoding='utf-8')


def _move_group(context, *args, **kwargs):
    urdf_file = LaunchConfiguration('robot_description_file').perform(context)
    namespace = LaunchConfiguration('namespace').perform(context)
    srdf_file = Path(get_package_share_directory('robot_moveit')) / 'config' / 'robot.srdf'

    parameters = [{
        # 描述：URDF 用 prepare_setup 生成的产物，SRDF 用本包的
        'robot_description': _text(urdf_file),
        'robot_description_semantic': _text(srdf_file),
        'robot_description_kinematics': {
            'arm': {
                'kinematics_solver': 'kdl_kinematics_plugin/KDLKinematicsPlugin',
                'kinematics_solver_search_resolution': 0.005,
                'kinematics_solver_timeout': 0.05,
            },
        },
        # 轨迹出口：直接对着第 3 步已验证的 JTC action。
        # 参数根名来自 moveit_simple_controller_manager.cpp（2.5.10）里的
        # `PARAM_BASE_NAME = "moveit_simple_controller_manager"`：控制器清单就挂在
        # `moveit_simple_controller_manager.controller_names`，**不是** `moveit_controllers.*`。
        # 写成后者不会报错，只会静默得到 "No controller_names specified"。
        'moveit_simple_controller_manager': CONTROLLERS,
        'planning_pipelines': ['ompl'],
        'default_planning_pipeline': 'ompl',
        'ompl': {
            'planning_plugin': 'ompl_interface/OMPLPlanner',
            'request_adapters': ADAPTERS,
            'start_state_max_bounds_error': 0.1,
        },
        'use_sim_time': True,
    }]

    return [Node(
        package='moveit_ros_move_group',
        executable='move_group',
        namespace=namespace,
        output='screen',
        parameters=parameters,
        # TF 与关节状态都在 a200_0000 命名空间下（Clearpath 的命名），必须显式重映射
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static'),
                    ('joint_states', 'platform/joint_states')],
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='a200_0000'),
        DeclareLaunchArgument(
            'robot_description_file',
            default_value='/home/ros/.q2q/setup/a200_kinova/robot.patched.urdf',
            description='prepare_setup.py 生成的 URDF（VM 上的生成产物，不是源码）'),
        OpaqueFunction(function=_move_group),
    ])
