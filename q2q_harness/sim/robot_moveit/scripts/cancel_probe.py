"""取消路径探针：分清「控制器不认取消」和「MoveIt 的 MoveGroup 不转发取消」。

用法（VM 内，先 source ~/.q2q/step3_env.sh）：

    python3 cancel_probe.py --mode jtc      # 直接给 JTC 发 3 s 轨迹，0.6 s 后取消
    python3 cancel_probe.py --mode moveit   # MoveIt 规划 → 自己发 JTC → 0.6 s 后取消

输出：是否被接受、取消前后关节位置与速度、1 s 后是否还在动。
运动幅度固定为 j1 到 0.8 rad、3 秒走完，只测取消，不做别的动作。
"""

import argparse
import json
import time

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from moveit_msgs.srv import GetMotionPlan
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

JOINTS = ['arm_0_joint_%d' % i for i in range(1, 7)]
JTC = '/a200_0000/arm_0_joint_trajectory_controller/follow_joint_trajectory'
PLAN = '/a200_0000/plan_kinematic_path'
TARGET = [0.8, 0.0, 0.0, 0.0, 0.0, 0.0]


class Probe(Node):
    def __init__(self):
        super().__init__('cancel_probe')
        self.pos, self.vel = {}, {}
        self.create_subscription(JointState, '/a200_0000/platform/joint_states', self._on, 10)
        self.client = ActionClient(self, FollowJointTrajectory, JTC)
        self.plan_client = self.create_client(GetMotionPlan, PLAN)

    def _on(self, msg):
        self.pos = dict(zip(msg.name, msg.position))
        self.vel = dict(zip(msg.name, msg.velocity))

    def snap(self):
        return {'pos': [round(self.pos.get(j, float('nan')), 3) for j in JOINTS],
                'vel': [round(self.vel.get(j, float('nan')), 3) for j in JOINTS]}


def simple_trajectory():
    point = JointTrajectoryPoint(positions=TARGET, velocities=[0.0] * 6,
                                 time_from_start=Duration(sec=3))
    return JointTrajectory(joint_names=JOINTS, points=[point])


def moveit_trajectory(node):
    from moveit_msgs.msg import Constraints, JointConstraint, MotionPlanRequest
    req = MotionPlanRequest()
    req.group_name = 'arm'
    req.num_planning_attempts = 5
    req.allowed_planning_time = 5.0
    req.max_velocity_scaling_factor = 0.3
    req.max_acceleration_scaling_factor = 0.3
    req.goal_constraints = [Constraints(joint_constraints=[
        JointConstraint(joint_name=j, position=p, tolerance_above=0.02,
                        tolerance_below=0.02, weight=1.0)
        for j, p in zip(JOINTS, TARGET)])]

    request = GetMotionPlan.Request()
    request.motion_plan_request = req
    future = node.plan_client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=20.0)
    return future.result().motion_plan_response.trajectory.joint_trajectory


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--mode', choices=['jtc', 'moveit'], default='jtc')
    parser.add_argument('--hold', type=float, default=0.6)
    args = parser.parse_args()

    rclpy.init()
    node = Probe()
    node.client.wait_for_server(timeout_sec=10.0)
    for _ in range(10):
        rclpy.spin_once(node, timeout_sec=0.1)

    trajectory = simple_trajectory() if args.mode == 'jtc' else moveit_trajectory(node)
    if not trajectory.points:
        print(json.dumps({'error': 'no trajectory'}, ensure_ascii=False))
        return 1

    before = node.snap()
    send = node.client.send_goal_async(FollowJointTrajectory.Goal(trajectory=trajectory))
    rclpy.spin_until_future_complete(node, send, timeout_sec=10.0)
    handle = send.result()

    deadline = time.monotonic() + args.hold
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
    at_cancel = node.snap()

    started = time.monotonic()
    cancel = handle.cancel_goal_async()
    rclpy.spin_until_future_complete(node, cancel, timeout_sec=10.0)
    cancel_latency = round(time.monotonic() - started, 3)

    time.sleep(1.0)
    for _ in range(10):
        rclpy.spin_once(node, timeout_sec=0.1)
    after = node.snap()

    print(json.dumps({
        'mode': args.mode,
        'accepted': bool(handle and handle.accepted),
        'cancel_latency_s': cancel_latency,
        'points': len(trajectory.points),
        'before': before,
        'at_cancel': at_cancel,
        'after_1s': after,
        'still_moving_1s_after_cancel': abs(after['pos'][0] - at_cancel['pos'][0]) > 0.02,
    }, ensure_ascii=False))
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
