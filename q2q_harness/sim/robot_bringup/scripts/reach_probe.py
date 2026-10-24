#!/usr/bin/env python3
"""reach_probe：把一条抓取观测的位姿交给 MoveIt 规划，回答「这个目标够得着吗」。

用法（VM 内，先 source ~/.q2q/step3_env.sh）：
    python3 reach_probe.py                 # 读一条观测，规划到它的 pose
    python3 reach_probe.py --z-offset 0.10 # 规划到观测点上方 10 cm（预抓取）

只发规划请求，不执行、不运动。失败时打印 MoveIt 的 error_code 与约束摘要。
"""

import argparse
import json
import time

import numpy as np

import grasp_pose
import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import (Constraints, OrientationConstraint, PositionConstraint,
                             MotionPlanRequest)
from moveit_msgs.srv import GetMotionPlan
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive
from q2q_interfaces.msg import GraspObservation

LINK = 'arm_0_end_effector_link'
FRAME = 'base_footprint'


class Probe(Node):
    def __init__(self):
        super().__init__('reach_probe')
        self.latest = None
        self.create_subscription(GraspObservation, '/a200_0000/perception/grasp_observation',
                                 self._on, 5)
        self.plan_client = self.create_client(GetMotionPlan, '/a200_0000/plan_kinematic_path')

    def _on(self, msg):
        if msg.valid:
            self.latest = msg


def build_request(observation, z_offset, approach=None):
    position = Pose()
    position.position.x = observation.pose.position.x
    position.position.y = observation.pose.position.y
    position.position.z = observation.pose.position.z + z_offset

    region = PositionConstraint()
    region.header.frame_id = FRAME
    region.link_name = LINK
    sphere = SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[0.02])
    region.constraint_region.primitives = [sphere]
    region.constraint_region.primitive_poses = [position]
    region.weight = 1.0

    orientation = OrientationConstraint()
    orientation.header.frame_id = FRAME
    orientation.link_name = LINK
    if approach is None:
        orientation.orientation = observation.pose.orientation
    else:
        rotation = grasp_pose.rotation_from_approach(approach)
        quat = grasp_pose.rotation_to_quaternion(rotation)
        orientation.orientation.x, orientation.orientation.y = float(quat[0]), float(quat[1])
        orientation.orientation.z, orientation.orientation.w = float(quat[2]), float(quat[3])
    orientation.absolute_x_axis_tolerance = 0.2
    orientation.absolute_y_axis_tolerance = 0.2
    orientation.absolute_z_axis_tolerance = 0.2
    orientation.weight = 1.0

    request = MotionPlanRequest()
    request.group_name = 'arm'
    request.num_planning_attempts = 5
    request.allowed_planning_time = 5.0
    request.max_velocity_scaling_factor = 0.2
    request.max_acceleration_scaling_factor = 0.2
    request.goal_constraints = [Constraints(position_constraints=[region],
                                            orientation_constraints=[orientation])]
    wrapper = GetMotionPlan.Request()
    wrapper.motion_plan_request = request
    return wrapper


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--z-offset', type=float, default=0.0)
    parser.add_argument('--approach', choices=['camera', 'topdown', 'forward'], default='camera')
    args = parser.parse_args()

    rclpy.init()
    node = Probe()
    deadline = time.monotonic() + 20.0
    while node.latest is None and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
    if node.latest is None:
        print(json.dumps({'error': '没收到有效观测'}, ensure_ascii=False))
        return 1
    if not node.plan_client.wait_for_service(timeout_sec=15.0):
        print(json.dumps({'error': 'plan_kinematic_path 不在'}, ensure_ascii=False))
        return 1

    vectors = {'topdown': [0.0, 0.0, -1.0], 'forward': [1.0, 0.0, 0.0]}
    request = build_request(node.latest, args.z_offset, vectors.get(args.approach))
    future = node.plan_client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=30.0)
    response = future.result()
    plan = response.motion_plan_response
    print(json.dumps({
        'target_position': [round(request.motion_plan_request.goal_constraints[0]
                                  .position_constraints[0].constraint_region
                                  .primitive_poses[0].position.x, 4),
                            round(request.motion_plan_request.goal_constraints[0]
                                  .position_constraints[0].constraint_region
                                  .primitive_poses[0].position.y, 4),
                            round(request.motion_plan_request.goal_constraints[0]
                                  .position_constraints[0].constraint_region
                                  .primitive_poses[0].position.z, 4)],
        'error_code': plan.error_code.val,
        'planning_time_s': round(plan.planning_time, 3),
        'points': len(plan.trajectory.joint_trajectory.points),
    }, ensure_ascii=False))
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
