#!/usr/bin/env python3
"""arm_executor：把「MoveIt 规划」和「JTC 执行」包成一个可取消的 action。

接口（第 4 步敲定，不新建 msg）：
  action  /a200_0000/arm_executor/move   control_msgs/action/FollowJointTrajectory
  goal.trajectory 只有 1 个点 → 当作目标关节位，先规划再执行
  goal.trajectory 有多个点        → 直接透传给 JTC
  result.error_code  0 成功 / 1 规划失败 / 2 执行失败 / 3 已取消

为什么不用 move_group 的 MoveGroup action：2026-10-22 实测它取消要 4.65 s 且仍走到目标
（cancel 回调只 ACCEPT，真正 stop 要等执行回调让出线程）；JTC 层取消 0.002 s 生效。
所以这里自己做「规划 → 发 action → 转发取消」。
"""

import math
import threading
import rclpy
from control_msgs.action import FollowJointTrajectory
from moveit_msgs.srv import GetMotionPlan
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

import arm_plan

ACTION = '/a200_0000/arm_executor/move'
JTC = '/a200_0000/arm_0_joint_trajectory_controller/follow_joint_trajectory'


class ArmExecutor(Node):
    def __init__(self):
        super().__init__('arm_executor')
        group = ReentrantCallbackGroup()
        self.plan_client = self.create_client(GetMotionPlan, arm_plan.PLAN_SERVICE,
                                              callback_group=group)
        self.jtc = ActionClient(self, FollowJointTrajectory, JTC, callback_group=group)
        self.jtc_handle = None
        self.busy, self.faulted, self.lock = False, False, threading.Lock()
        self.server = ActionServer(self, FollowJointTrajectory, ACTION,
                                   execute_callback=self.execute,
                                   goal_callback=self.accept,
                                   cancel_callback=self.on_cancel, callback_group=group)
        self.get_logger().info('arm_executor 就绪：%s' % ACTION)

    def accept(self, request):
        t = request.trajectory
        with self.lock:
            valid = (not self.busy and not self.faulted and self.jtc.server_is_ready() and bool(t.points)
                     and len(t.joint_names) == 6 and set(t.joint_names) == set(arm_plan.JOINTS)
                     and all(len(p.positions) == 6 and all(math.isfinite(v) for v in p.positions)
                             for p in t.points))
            if valid:
                self.busy = True
            return GoalResponse.ACCEPT if valid else GoalResponse.REJECT

    def on_cancel(self, _goal_handle):
        """取消原样转给 JTC；本节点不自己刹车。"""
        if self.jtc_handle is not None:
            self.jtc_handle.cancel_goal_async()
        return CancelResponse.ACCEPT

    async def execute(self, goal_handle):
        try:
            return await self.execute_one(goal_handle)
        except Exception as error:
            self.get_logger().error(str(error))
            if self.jtc_handle is not None:
                try:
                    await arm_plan.bounded(self, self.jtc_handle.cancel_goal_async(), 3.)
                    await arm_plan.bounded(self, self.jtc_handle.get_result_async(), 5.)
                except Exception as cancel_error:
                    self.faulted = True
                    self.get_logger().error('STOP UNCONFIRMED: '+str(cancel_error))
            return self._finish(goal_handle, 2)
        finally:
            with self.lock:
                self.jtc_handle, self.busy = None, False

    async def execute_one(self, goal_handle):
        trajectory = goal_handle.request.trajectory
        if not trajectory.points:
            return self._finish(goal_handle, 1)
        if len(trajectory.points) == 1:
            trajectory, why = await arm_plan.plan(
                self.plan_client, [trajectory.points[0].positions[trajectory.joint_names.index(j)]
                                   for j in arm_plan.JOINTS], 0.3, self)
            if trajectory is None:
                self.get_logger().warn('规划没成功：%s' % why)
                return self._finish(goal_handle, 1)
        if goal_handle.is_cancel_requested:
            return self._finish(goal_handle, 3)

        pending = self.jtc.send_goal_async(
            FollowJointTrajectory.Goal(trajectory=trajectory),
            feedback_callback=lambda msg: goal_handle.publish_feedback(msg.feedback))
        try:
            self.jtc_handle = await arm_plan.bounded(self, pending, 8.)
        except TimeoutError:
            self.faulted = True
            pending.add_done_callback(lambda f: f.result().cancel_goal_async()
                if not f.exception() and f.result() and f.result().accepted else None)
            raise
        if not self.jtc_handle.accepted:
            return self._finish(goal_handle, 2)
        if goal_handle.is_cancel_requested:
            await arm_plan.bounded(self, self.jtc_handle.cancel_goal_async(), 3.)

        # 直接等 JTC 的结果：rclpy 的 Future 可以 await，但没有 asyncio 事件循环，
        # 所以这里不能 sleep 轮询。取消由 on_cancel 直接转发给 JTC。
        duration = trajectory.points[-1].time_from_start
        budget = min(180., 3*(duration.sec+duration.nanosec/1e9)+15)
        completed = await arm_plan.bounded(self, self.jtc_handle.get_result_async(), budget)
        if goal_handle.is_cancel_requested and completed.status == 5:
            return self._finish(goal_handle, 3)
        failed = completed.status != 4 or completed.result.error_code != 0
        return self._finish(goal_handle, 2 if failed else 0)

    def _finish(self, goal_handle, code):
        result = FollowJointTrajectory.Result()
        result.error_code = code
        if code == 0:
            goal_handle.succeed()
        elif code == 3:
            goal_handle.canceled()
        else:
            goal_handle.abort()
        return result


def main():
    rclpy.init()
    node = ArmExecutor()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
