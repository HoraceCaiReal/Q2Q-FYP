#!/usr/bin/env python3
"""Truth-assisted final straight docking for this test station, not warehouse navigation.

Run only after Nav2 has terminated and the arm is stowed. Bounded by fresh physical
odometry, yaw/lateral limits and a wall deadline; always publish stop on exit.
"""
import argparse
import json
import math
import time
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--x', type=float, default=.88)
    # 墙钟窗口：默认 1.5 s。原来写死 0.5 s，只在 RTF≈1 的世界里成立；
    # 5 ms 物理步长的场景（q2q_warehouse_grasp50）里 odom 墙钟间隔会被拉长，0.5 s 会误判 stale（10-24 实测）。
    parser.add_argument('--stale-s', type=float, default=1.5)
    args = parser.parse_args()
    if not .75 <= args.x <= .89:
        parser.error('Station x must stay within [.75,.89], clear of table at x=1.15')
    rclpy.init()
    node = Node('station_approach')
    state = {}
    node.create_subscription(Odometry, '/a200_0000/ground_truth/odom',
                             lambda m: state.update(msg=m, at=time.monotonic()), 10)
    pub = node.create_publisher(Twist, '/a200_0000/cmd_vel', 10)
    report = dict(passed=False, target_x=args.x)
    try:
        deadline = time.monotonic()+60
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.05)
            if 'msg' not in state:
                pub.publish(Twist())
                continue
            if time.monotonic()-state['at'] > args.stale_s:
                raise RuntimeError('Physical odometry stale')
            p, q = state['msg'].pose.pose.position, state['msg'].pose.pose.orientation
            yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
            if abs(yaw) > .04 or abs(p.y) > .02 or p.x > .90:
                raise RuntimeError('Station approach outside straight corridor')
            error = args.x-p.x
            if error <= .003:
                report.update(passed=abs(error)<.006, final_x=p.x)
                break
            cmd = Twist()
            cmd.linear.x = min(.08, max(.012, error*.8))
            pub.publish(cmd)
        else:
            raise TimeoutError('Station approach exceeded 60 seconds')
    except Exception as error:
        report['error'] = str(error)
    finally:
        for _ in range(10):
            pub.publish(Twist())
            rclpy.spin_once(node, timeout_sec=.05)
        print(json.dumps(report), flush=True)
        node.destroy_node()
        rclpy.shutdown()
    return 0 if report['passed'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
