#!/usr/bin/env python3
"""Empty-gripper regression: terminal actions AND physical gaps, both directions.

The .90-rad soft limit leaves roughly 27 mm free gap; max_effort is not a force limit.
Run before moving the arm into contact. A timeout fails rather than triggering a retry.
"""
import argparse
import json
import time
from pathlib import Path
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import grasp_io


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--cycles', type=int, default=8)
    parser.add_argument('--effort', type=float, default=5.)
    parser.add_argument('--output')
    args = parser.parse_args()
    if args.cycles < 1:
        parser.error('--cycles must be positive')
    rclpy.init()
    node = Node('gripper_cycle')
    state = {}
    node.create_subscription(JointState, '/a200_0000/platform/joint_states',
                             lambda m: state.update(msg=m, received=time.monotonic()), 10)
    report = dict(passed=False, cycles_requested=args.cycles, history=[])
    try:
        grip = grasp_io.gripper_client(node)
        if grip is None:
            raise RuntimeError('gripper_cmd unavailable; check facade and JTC separately')
        deadline = time.monotonic()+3.
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
        previous = None
        for cycle in range(1, args.cycles+1):
            record = dict(cycle=cycle)
            report['history'].append(record)
            for tag, target, low, high in [('open', 0., .100, .120),
                                          ('close', .90, .020, .035)]:
                result = grasp_io.gripper(node, grip, target, args.effort)
                gap = grasp_io.jaw(grasp_io.read())[1]
                record[tag] = dict(result=result, gap_mm=gap*1000)
                if result['status'] != 4 or not result['reached'] or not low <= gap <= high:
                    raise RuntimeError('Failed terminal state or physical gap: '+tag)
                if previous is not None and abs(gap-previous) < .050:
                    raise RuntimeError('Insufficient physical travel: '+tag)
                previous = gap
            print(json.dumps(record), flush=True)
        report['passed'] = True
    except Exception as error:
        report['error'] = str(error)
    finally:
        if args.output:
            Path(args.output).write_text(json.dumps(report, indent=2), encoding='utf-8')
        print(json.dumps(report), flush=True)
        node.destroy_node()
        rclpy.shutdown()
    return 0 if report['passed'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
