"""验证 arm_executor：正常到位一次 + 取消一次。

用法（VM 内，先 source ~/.q2q/step3_env.sh）：
    python3 executor_probe.py --target 0.4 0 0 0 0 0
    python3 executor_probe.py --target 1.2 0 0 0 0 0 --cancel-after 0.6
"""

import argparse
import json
import time

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

JOINTS = ['arm_0_joint_%d' % i for i in range(1, 7)]
ACTION = '/a200_0000/arm_executor/move'
STATUS = {4: 'SUCCEEDED', 5: 'CANCELED', 6: 'ABORTED'}


class Probe(Node):
    def __init__(self):
        super().__init__('executor_probe')
        self.pos, self.vel = {}, {}
        self.create_subscription(JointState, '/a200_0000/platform/joint_states',
                                 self._on, 10)
        self.client = ActionClient(self, FollowJointTrajectory, ACTION)

    def _on(self, msg):
        self.pos = dict(zip(msg.name, msg.position))
        self.vel = dict(zip(msg.name, msg.velocity))

    def snap(self):
        return {'pos': [round(self.pos.get(j, float('nan')), 3) for j in JOINTS],
                'vel': [round(self.vel.get(j, float('nan')), 3) for j in JOINTS]}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--target', nargs=6, type=float, required=True)
    parser.add_argument('--cancel-after', type=float, default=0.0)
    args = parser.parse_args()

    rclpy.init()
    node = Probe()
    if not node.client.wait_for_server(timeout_sec=10.0):
        print(json.dumps({'error': 'arm_executor 不在'}, ensure_ascii=False))
        return 1
    for _ in range(10):
        rclpy.spin_once(node, timeout_sec=0.1)
    before = node.snap()

    goal = FollowJointTrajectory.Goal()
    point = JointTrajectoryPoint(positions=args.target)
    goal.trajectory = JointTrajectory(joint_names=JOINTS, points=[point])
    send = node.client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, send, timeout_sec=10.0)
    handle = send.result()

    cancel_latency = None
    if args.cancel_after > 0:
        deadline = time.monotonic() + args.cancel_after
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        started = time.monotonic()
        cancel = handle.cancel_goal_async()
        rclpy.spin_until_future_complete(node, cancel, timeout_sec=10.0)
        cancel_latency = round(time.monotonic() - started, 3)

    result = handle.get_result_async()
    rclpy.spin_until_future_complete(node, result, timeout_sec=90.0)
    wrapped = result.result()
    time.sleep(1.0)
    for _ in range(10):
        rclpy.spin_once(node, timeout_sec=0.1)

    print(json.dumps({
        'accepted': bool(handle and handle.accepted),
        'status': STATUS.get(wrapped.status, wrapped.status) if wrapped else None,
        'error_code': wrapped.result.error_code if wrapped else None,
        'cancel_latency_s': cancel_latency,
        'target': args.target,
        'before': before,
        'after': node.snap(),
    }, ensure_ascii=False))
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
