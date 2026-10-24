#!/usr/bin/env python3
"""GripperCommand facade over a bounded-speed JTC; position simulation, not force control."""
import threading
import time
import os
from pathlib import Path
import xml.etree.ElementTree as ET
import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from control_msgs.action import GripperCommand, FollowJointTrajectory
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from action_wait import bounded
from rclpy.clock import Clock, ClockType

JOINT = 'arm_0_gripper_right_finger_bottom_joint'
NS = '/a200_0000'


class Gripper(Node):
    def __init__(self):
        super().__init__('gripper_action')
        self.position, self.received, self.busy = None, 0., False
        self.velocity, self.state_stamp = 0., 0
        self.commanded = os.environ.get('Q2Q_GRIPPER_MODE', 'mimic') == 'commanded'
        self.command_joints, self.factors = [JOINT], [1.]
        if self.commanded:
            root = ET.parse(Path.home()/'.q2q/setup/a200_kinova/robot.patched.urdf').getroot()
            joints = {j.get('name'): j for j in root.findall('joint')}
            names = sorted(n for n in joints if 'gripper' in n and joints[n].find('mimic') is not None)
            def factor(name, seen=()):
                if name == JOINT:
                    return 1.
                if name in seen:
                    raise ValueError('Cyclic mimic chain')
                mimic = joints[name].find('mimic')
                if abs(float(mimic.get('offset', '0'))) > 1e-9:
                    raise ValueError('Command coupling requires baked zero offsets')
                return float(mimic.get('multiplier', '1'))*factor(mimic.get('joint'), seen+(name,))
            self.command_joints += names
            self.factors += [factor(n) for n in names]
            if len(self.command_joints) != 4:
                raise ValueError('Expected four gripper joints')
        self.faulted = False
        self.lock, self.jtc_handle = threading.Lock(), None
        group = ReentrantCallbackGroup()
        self.create_subscription(JointState, NS+'/platform/joint_states', self.state, 10,
                                 callback_group=group)
        self.client = ActionClient(self, FollowJointTrajectory,
            NS+'/arm_0_gripper_controller/follow_joint_trajectory', callback_group=group)
        self.server = ActionServer(self, GripperCommand,
            NS+'/arm_0_gripper_controller/gripper_cmd', self.execute,
            goal_callback=self.goal, cancel_callback=self.cancel, callback_group=group)
        self.get_logger().info('Bounded JTC gripper; max_effort is not enforced in position mode')

    def state(self, msg):
        if JOINT in msg.name:
            self.position = msg.position[msg.name.index(JOINT)]
            index = msg.name.index(JOINT)
            self.velocity = msg.velocity[index] if len(msg.velocity) > index else float('inf')
            self.state_stamp = msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
            self.received = time.monotonic()

    def goal(self, request):
        with self.lock:
            valid = (not self.busy and not self.faulted and self.position is not None and
                     time.monotonic()-self.received < 1 and
                     0 <= request.command.position <= .90 and self.client.server_is_ready())
            if valid:
                self.busy = True
            return GoalResponse.ACCEPT if valid else GoalResponse.REJECT

    def cancel(self, _):
        if self.jtc_handle:
            self.jtc_handle.cancel_goal_async()
        return CancelResponse.ACCEPT

    async def execute(self, goal):
        result = GripperCommand.Result()
        pending = None
        blocked_contact = False
        try:
            initial = self.position
            seconds = max(1., abs(goal.request.command.position-self.position)/.30)
            duration = Duration(sec=int(seconds), nanosec=int(seconds % 1*1e9))
            trajectory = JointTrajectory(joint_names=self.command_joints, points=[JointTrajectoryPoint(
                positions=[goal.request.command.position*f for f in self.factors], time_from_start=duration)])
            pending = self.client.send_goal_async(FollowJointTrajectory.Goal(trajectory=trajectory))
            self.jtc_handle = await bounded(self, pending, 8.)
            if not self.jtc_handle or not self.jtc_handle.accepted:
                raise RuntimeError('JTC rejected goal')
            if goal.is_cancel_requested:
                await bounded(self, self.jtc_handle.cancel_goal_async(), 3.)
            terminal_future = self.jtc_handle.get_result_async()
            outcome = rclpy.task.Future()
            stable_since = [None]
            def monitor():
                if outcome.done():
                    return
                if terminal_future.done():
                    if terminal_future.exception():
                        outcome.set_exception(terminal_future.exception())
                    else:
                        outcome.set_result(terminal_future.result())
                    return
                # Detect blocked *physical* motion, not the trajectory's commanded
                # angle. Require motion first and advancing simulation timestamps.
                blocked = (not self.commanded and goal.request.command.position > initial and
                    self.position-initial > .015 and abs(self.velocity) < .02 and
                    goal.request.command.position-self.position > .03 and
                    time.monotonic()-self.received < 1.)
                if not blocked:
                    stable_since[0] = None
                elif stable_since[0] is None:
                    stable_since[0] = self.state_stamp
                elif self.state_stamp-stable_since[0] >= 150000000:
                    outcome.set_result(None)
            watch = self.create_timer(.05, monitor,
                clock=Clock(clock_type=ClockType.STEADY_TIME))
            try:
                terminal = await bounded(self, outcome, 45.)
            finally:
                self.destroy_timer(watch)
            if terminal is None and not goal.is_cancel_requested:
                # 物理上被挡住了：这是**夹到东西**的正常终态，不是失败（10-24 最后一次修）
                blocked_contact = True
                self.get_logger().info('Contact stall: cancel ramp and hold with 0.012 rad preload')
                await bounded(self, self.jtc_handle.cancel_goal_async(), 3.)
                stopped = await bounded(self, terminal_future, 5.)
                if stopped.status not in (4, 5):
                    raise RuntimeError('Contact stop not confirmed')
                preload = min(.90, self.position+.012)
                hold = FollowJointTrajectory.Goal(trajectory=JointTrajectory(joint_names=[JOINT],
                    points=[JointTrajectoryPoint(positions=[preload], time_from_start=Duration(sec=1))]))
                self.jtc_handle = None
                pending = self.client.send_goal_async(hold)
                self.jtc_handle = await bounded(self, pending, 8.)
                if not self.jtc_handle or not self.jtc_handle.accepted:
                    raise RuntimeError('Contact hold rejected')
                if goal.is_cancel_requested:
                    await bounded(self, self.jtc_handle.cancel_goal_async(), 3.)
                terminal = await bounded(self, self.jtc_handle.get_result_async(), 30.)
            elif terminal is None:
                terminal = await bounded(self, terminal_future, 5.)
            # JTC deliberately permits blocked contact. Its terminal event can precede
            # physical settling, so do not classify lag as a stalled gripper.
            position_before_settle = self.position
            settled = rclpy.task.Future()
            timer = self.create_timer(1., lambda: settled.set_result(True)
                if not settled.done() else None, clock=Clock(clock_type=ClockType.STEADY_TIME))
            try:
                await bounded(self, settled, 2.)
            finally:
                self.destroy_timer(timer)
            result.position = float(self.position)
            result.reached_goal = abs(self.position-goal.request.command.position) < .025
            result.stalled = not result.reached_goal
            # 判定顺序：取消 → 正常到位 → **被物体挡住的接触终态** → 其余才算失败。
            # 为什么要有第三条：接触保持的 JTC 终态天然是 ABORTED（目标角永远到不了），
            # 把它当失败会让「明明夹住了」在门禁里变成失败（10-24 实测 status=6）。
            fresh_state = time.monotonic()-self.received < 1.
            # 停稳用**位移**判，不用 velocity 字段：关节状态里可能没有速度分量，
            # 那时 velocity 被读成 inf，「停住」永远不成立（10-24 实测 status=6 就是这么漏的）。
            moved = abs(self.position - position_before_settle)
            # 阈值 0.05 rad ≈ 2.9°：被箱子顶住时手指在 2 s 里还会慢慢蠕变几毫弧度，
            # 用 0.015 会把「已经夹住」判成没停稳（10-24 实测）。
            still = fresh_state and moved < .05
            # 「接触」的可核对定义：**确实朝闭合方向走了 >0.015 rad，而且真的停住了**。
            # 不要求 JTC 报 SUCCEEDED——被物体挡住时目标角永远到不了，JTC 的终态
            # 天然就是 ABORTED（实测 status=6）。把这种「走了一段又停住」当失败，
            # 会让明明夹住的结果在门禁里变成失败，这是 10-24 最后一次修的那个语义 bug。
            closed_enough = (not self.commanded and goal.request.command.position > initial
                             and self.position - initial > .015)
            contact_stop = (blocked_contact or closed_enough) and still
            if goal.is_cancel_requested and terminal.status == 5:
                goal.canceled()
            elif terminal.status == 4 and fresh_state:
                goal.succeed()
            elif contact_stop:
                self.get_logger().info('Contact stop: closed %.3f rad and stopped (JTC status %d counts as contact, not failure)'
                                       % (self.position-initial, terminal.status))
                goal.succeed()
            else:
                goal.abort()
        except Exception as error:
            self.get_logger().error(str(error))
            if self.position is not None:
                result.position = float(self.position)
                result.stalled = True
            if self.jtc_handle:
                try:
                    await bounded(self, self.jtc_handle.cancel_goal_async(), 3.)
                    await bounded(self, self.jtc_handle.get_result_async(), 5.)
                except Exception as cancel_error:
                    self.faulted = True
                    self.get_logger().error('Cancellation unconfirmed: '+str(cancel_error))
            elif pending:
                self.faulted = True
                pending.add_done_callback(lambda f: f.result().cancel_goal_async()
                    if f.result() and f.result().accepted else None)
            goal.abort()
        finally:
            with self.lock:
                self.jtc_handle = None
                self.busy = False
        return result


def main():
    rclpy.init()
    node = Gripper()
    # All waits yield rclpy Futures. One executor thread also prevents Humble's
    # wait-set/destroy_timer race while an asynchronous goal is completing.
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
