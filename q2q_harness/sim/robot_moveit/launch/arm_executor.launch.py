"""启动 arm_executor：需要 move_group 已在跑（它只负责规划）。"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='a200_0000'),
        Node(
            package='robot_moveit',
            executable='arm_executor.py',
            namespace=LaunchConfiguration('namespace'),
            output='screen',
            parameters=[{'use_sim_time': True}],
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
        ),
    ])
