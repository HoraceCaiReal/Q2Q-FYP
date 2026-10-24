#!/usr/bin/env python3
"""ik_sweep：只读扫「手能到哪」——沿 +x 与高度取格，逐点问 compute_ik 有没有解。

用法（VM 内，先 source ~/.q2q/step3_env.sh）：
    python3 ik_sweep.py                          # 默认：自上而下姿态，y=0.17
    python3 ik_sweep.py --approach camera --y 0.0

不发任何运动：只调 `/a200_0000/compute_ik`。输出每个格子的 error_code（1 = 有解，-31 = 无解）。
"""

import argparse
import json

import numpy as np

import grasp_pose
import rclpy
from moveit_msgs.msg import PositionIKRequest, RobotState
from moveit_msgs.srv import GetPositionIK
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

APPROACH = {'topdown': [0.0, 0.0, -1.0], 'forward': [1.0, 0.0, 0.0]}


class Sweep(Node):
    def __init__(self):
        super().__init__('ik_sweep')
        self.client = self.create_client(GetPositionIK, '/a200_0000/compute_ik')

    def ik(self, position, quaternion):
        request = GetPositionIK.Request()
        inner = PositionIKRequest()
        inner.group_name = 'arm'
        inner.robot_state = RobotState()
        inner.avoid_collisions = False
        pose = PoseStamped()
        pose.header.frame_id = 'base_footprint'
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = position
        (pose.pose.orientation.x, pose.pose.orientation.y,
         pose.pose.orientation.z, pose.pose.orientation.w) = quaternion
        inner.pose_stamped = pose
        inner.timeout.sec = 2
        request.ik_request = inner
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=15.0)
        result = future.result()
        return result.error_code.val if result else -99999


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--approach', choices=['topdown', 'forward'], default='topdown')
    parser.add_argument('--y', type=float, default=0.17)
    parser.add_argument('--x', nargs=3, type=float, default=[0.25, 0.95, 0.10])
    parser.add_argument('--z', nargs=3, type=float, default=[0.15, 1.20, 0.15])
    args = parser.parse_args()

    quaternion = tuple(grasp_pose.rotation_to_quaternion(
        grasp_pose.rotation_from_approach(np.array(APPROACH[args.approach]))))

    rclpy.init()
    node = Sweep()
    if not node.client.wait_for_service(timeout_sec=15.0):
        print(json.dumps({'error': 'compute_ik 不在'}, ensure_ascii=False))
        return 1

    xs = np.arange(args.x[0], args.x[1] + 1e-9, args.x[2])
    zs = np.arange(args.z[0], args.z[1] + 1e-9, args.z[2])
    grid, ok_count = [], 0
    for z in zs:
        row = []
        for x in xs:
            code = node.ik((float(x), args.y, float(z)), quaternion)
            row.append(code)
            ok_count += code == 1
        grid.append({'z': round(float(z), 3),
                     'codes': row, 'ok': [c == 1 for c in row]})
    print(json.dumps({
        'approach': args.approach,
        'y': args.y,
        'xs': [round(float(x), 3) for x in xs],
        'zs': [round(float(z), 3) for z in zs],
        'solved': ok_count,
        'total': len(xs) * len(zs),
        'grid': grid,
    }, ensure_ascii=False))
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
