#!/usr/bin/env python3
"""读规划场景并做状态可行性 A/B：同一个关节状态，在「有测试障碍 / 无」两种场景下是否可行。

用法（VM 内，先 source ~/.q2q/step3_env.sh）：
    python3 scene_probe.py --count
    python3 scene_probe.py --sweep      # 扫 j2，打印每个状态的 valid

只读，不发运动。
"""

import argparse
import json

import rclpy
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetPlanningScene, GetStateValidity
from rclpy.node import Node
from sensor_msgs.msg import JointState

JOINTS = ['arm_0_joint_%d' % i for i in range(1, 7)]
SCENE = '/a200_0000/get_planning_scene'
VALID = '/a200_0000/check_state_validity'


def call(node, client, request, timeout=15.0):
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    return future.result() if future.done() else None


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--count', action='store_true')
    parser.add_argument('--sweep', action='store_true')
    args = parser.parse_args()

    rclpy.init()
    node = Node('scene_probe')
    scene_client = node.create_client(GetPlanningScene, SCENE)
    valid_client = node.create_client(GetStateValidity, VALID)
    scene_client.wait_for_service(timeout_sec=15.0)
    valid_client.wait_for_service(timeout_sec=15.0)

    summary = {}
    if args.count:
        request = GetPlanningScene.Request()
        request.components.components = 0
        response = call(node, scene_client, request)
        names = [o.id for o in response.scene.world.collision_objects]
        summary['objects'] = len(names)
        summary['names'] = names

    if args.sweep:
        results = []
        for j2 in (-1.2, -0.8, -0.4, 0.0, 0.4, 0.8, 1.2):
            state = RobotState()
            state.joint_state = JointState(name=JOINTS,
                                           position=[0.0, j2, 0.0, 0.0, 0.0, 0.0])
            request = GetStateValidity.Request(robot_state=state, group_name='arm')
            response = call(node, valid_client, request)
            results.append({'j2': j2, 'valid': bool(response and response.valid),
                            'contacts': len(response.contacts) if response else -1})
        summary['sweep'] = results

    print(json.dumps(summary, ensure_ascii=False))
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
