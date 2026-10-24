#!/usr/bin/env python3
"""pose_move：按「末端位姿」规划并执行一次（MoveGroup action，规划+执行一体）。

用法（VM 内，先 source ~/.q2q/step3_env.sh）：
    python3 pose_move.py --x 0.46 --y 0.0 --z 0.685 --approach topdown
    python3 pose_move.py --x 0.46 --y 0.0 --z 0.565 --approach topdown --no-exec

只做一件事：把一个末端位姿送到。抓取流水线的 PREGRASP / APPROACH 用它。
"""

import argparse
import json
import time

import numpy as np

import grasp_pose
import rclpy
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, OrientationConstraint, PositionConstraint
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive

LINK = 'arm_0_end_effector_link'
FRAME = 'base_footprint'
JOINTS = ['arm_0_joint_%d' % i for i in range(1, 7)]
APPROACH = {'topdown': [0.0, 0.0, -1.0], 'forward': [1.0, 0.0, 0.0]}


class Mover(Node):
    def __init__(self):
        super().__init__('pose_move')
        self.pos = {}
        self.create_subscription(JointState, '/a200_0000/platform/joint_states', self._on, 10)
        self.client = ActionClient(self, MoveGroup, '/a200_0000/move_action')

    def _on(self, msg):
        self.pos = dict(zip(msg.name, msg.position))

    def snap(self):
        return [round(self.pos.get(j, float('nan')), 4) for j in JOINTS]


def build_goal(x, y, z, quaternion, plan_only, radius=0.015):
    region = PositionConstraint()
    region.header.frame_id = FRAME
    region.link_name = LINK
    region.constraint_region.primitives = [
        SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[radius])]
    region.weight = 1.0

    orientation = OrientationConstraint()
    orientation.header.frame_id = FRAME
    orientation.link_name = LINK
    (orientation.orientation.x, orientation.orientation.y,
     orientation.orientation.z, orientation.orientation.w) = quaternion
    orientation.absolute_x_axis_tolerance = 0.03
    orientation.absolute_y_axis_tolerance = 0.03
    orientation.absolute_z_axis_tolerance = 0.03
    orientation.weight = 1.0

    from geometry_msgs.msg import Pose
    pose = Pose()
    pose.orientation.w = 1.0
    pose.position.x, pose.position.y, pose.position.z = x, y, z
    region.constraint_region.primitive_poses = [pose]

    goal = MoveGroup.Goal()
    request = goal.request
    request.group_name = 'arm'
    request.num_planning_attempts = 1
    request.allowed_planning_time = 3.0
    request.start_state.is_diff = True
    request.max_velocity_scaling_factor = 0.2
    request.max_acceleration_scaling_factor = 0.2
    request.goal_constraints = [Constraints(position_constraints=[region],
                                            orientation_constraints=[orientation])]
    goal.planning_options.plan_only = plan_only
    return goal


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--x', type=float, required=True)
    parser.add_argument('--y', type=float, required=True)
    parser.add_argument('--z', type=float, required=True)
    parser.add_argument('--approach', choices=['topdown', 'forward'], default='topdown')
    parser.add_argument('--execute', action='store_true',
                        help='真的执行（默认只规划）。旧的直接执行路径会绕过 grasp_io 的有界 JTC，'
                             '所以默认关掉：这是诊断 CLI，不是抓取入口')
    parser.add_argument('--no-exec', action='store_true', help='（保留兼容）等价于默认行为')
    parser.add_argument('--jaw-horizontal', action='store_true',
                        help='绕接近轴再转 90°，让两指水平开合（实测默认姿态是竖直开合）')
    args = parser.parse_args()

    quaternion = grasp_pose.approach_quaternion(np.array(APPROACH[args.approach]),
                                                args.jaw_horizontal)

    rclpy.init()
    node = Mover()
    if not node.client.wait_for_server(timeout_sec=10.0):
        print(json.dumps({'error': 'move_action 不在'}, ensure_ascii=False))
        return 1
    for _ in range(10):
        rclpy.spin_once(node, timeout_sec=0.05)

    before = node.snap()
    plan_only = not (args.execute and not args.no_exec)
    send = node.client.send_goal_async(build_goal(args.x, args.y, args.z, quaternion, plan_only))
    rclpy.spin_until_future_complete(node, send, timeout_sec=10.0)
    handle = send.result()
    if handle is None or not handle.accepted:
        print(json.dumps({'accepted': False}, ensure_ascii=False))
        return 1
    result = handle.get_result_async()
    rclpy.spin_until_future_complete(node, result, timeout_sec=120.0)
    wrapped = result.result()
    time.sleep(1.0)
    for _ in range(10):
        rclpy.spin_once(node, timeout_sec=0.1)

    print(json.dumps({
        'target': [args.x, args.y, args.z],
        'approach': args.approach,
        'plan_only': plan_only,
        'action_status': wrapped.status if wrapped else None,
        'error_code': wrapped.result.error_code.val if wrapped else None,
        'planning_time_s': round(wrapped.result.planning_time, 3) if wrapped else None,
        'points': len(wrapped.result.planned_trajectory.joint_trajectory.points) if wrapped else 0,
        'joint_before': before,
        'joint_after': node.snap(),
    }, ensure_ascii=False))
    failed = wrapped is None or wrapped.result.error_code.val != 1
    node.destroy_node()
    rclpy.shutdown()
    return 1 if failed else 0


if __name__ == '__main__':
    raise SystemExit(main())
