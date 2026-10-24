"""第 4 步最小执行验证：一次小关节运动，可选中途取消。

用法（VM 内，先 source ~/.q2q/step3_env.sh，确认 move_group 在跑）：

    python3 ~/ros2_ws/src/robot_moveit/scripts/arm_smoke.py --target 0.3 0.2 0 0 0 0
    python3 ~/ros2_ws/src/robot_moveit/scripts/arm_smoke.py --target 0.8 0 0 0 0 0 --cancel-after 1.2

做的事：发一个 MoveGroup 目标（规划+执行），记录动作终态、规划耗时、轨迹点数，
以及执行前后 6 个手臂关节的实际位置；`--cancel-after` 时在 N 秒后取消并检查是否停住。
不伪造结果：拿不到 joint_states 就把 before/after 标成 null。
"""

import argparse
import json
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint
from sensor_msgs.msg import JointState

JOINTS = ['arm_0_joint_%d' % i for i in range(1, 7)]


class Smoke(Node):
    def __init__(self):
        super().__init__('arm_smoke')
        self.latest = None
        self.latest_vel = {}
        self.create_subscription(JointState, '/a200_0000/platform/joint_states', self._on_state, 10)
        self.client = ActionClient(self, MoveGroup, '/a200_0000/move_action')

    def _on_state(self, msg):
        self.latest = dict(zip(msg.name, msg.position))
        self.latest_vel = dict(zip(msg.name, msg.velocity))

    def arm_positions(self):
        if not self.latest:
            return None
        return [round(self.latest.get(j, float('nan')), 4) for j in JOINTS]

    def goal(self, target, cancel_after):
        constraints = [JointConstraint(joint_name=j, position=p, tolerance_above=0.02,
                                       tolerance_below=0.02, weight=1.0)
                       for j, p in zip(JOINTS, target)]
        request = MoveGroup.Goal().request
        request.group_name = 'arm'
        request.num_planning_attempts = 5
        request.allowed_planning_time = 5.0
        request.max_velocity_scaling_factor = 0.2
        request.max_acceleration_scaling_factor = 0.2
        request.goal_constraints = [Constraints(joint_constraints=constraints)]
        request.workspace_parameters.header.frame_id = 'base_footprint'
        goal = MoveGroup.Goal()
        goal.request = request
        return goal


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--target', nargs=6, type=float, required=True)
    parser.add_argument('--cancel-after', type=float, default=0.0)
    args = parser.parse_args()

    rclpy.init()
    node = Smoke()
    node.client.wait_for_server(timeout_sec=10.0)

    before = None
    deadline = time.monotonic() + 5.0
    while node.latest is None and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
    before = node.arm_positions()

    send = node.client.send_goal_async(node.goal(args.target, args.cancel_after))
    rclpy.spin_until_future_complete(node, send, timeout_sec=10.0)
    handle = send.result()
    if handle is None or not handle.accepted:
        print(json.dumps({'accepted': False}, ensure_ascii=False))
        return 1

    cancel_requested = False
    if args.cancel_after > 0:
        deadline = time.monotonic() + args.cancel_after
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        started = time.monotonic()
        cancel_future = handle.cancel_goal_async()
        rclpy.spin_until_future_complete(node, cancel_future, timeout_sec=5.0)
        cancel_requested = bool(cancel_future.done())
        cancel_error = None if cancel_future.done() is False else (
            str(cancel_future.exception()) if cancel_future.exception() else None)
        cancel_wait = round(time.monotonic() - started, 3)

    result_future = handle.get_result_async()
    rclpy.spin_until_future_complete(node, result_future, timeout_sec=60.0)
    wrapped = result_future.result()
    time.sleep(1.0)  # 留出停稳窗口
    for _ in range(5):
        rclpy.spin_once(node, timeout_sec=0.1)

    summary = {
        'accepted': True,
        'cancel_requested': cancel_requested,
        'cancel_wait_s': locals().get('cancel_wait'),
        'cancel_error': locals().get('cancel_error'),
        'action_status': wrapped.status if wrapped else None,
        'error_code': wrapped.result.error_code.val if wrapped else None,
        'planning_time': round(wrapped.result.planning_time, 3) if wrapped else None,
        'points': len(wrapped.result.planned_trajectory.joint_trajectory.points) if wrapped else 0,
        'before': before,
        'after': node.arm_positions(),
        'after_velocity': [round(node.latest_vel.get(j, float('nan')), 4) for j in JOINTS],
        'target': args.target,
    }
    print(json.dumps(summary, ensure_ascii=False))
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
