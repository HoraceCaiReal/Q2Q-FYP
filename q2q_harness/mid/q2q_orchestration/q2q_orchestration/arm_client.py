"""机械臂的两个基础动作：回收到收拢位（stow）、松爪（release）。

为什么单独一个文件：这两个动作都很短，但都要处理「异步 + 取消 + 结果」，
塞进节点会把状态机那部分挤没。约定：只发一次，不重发；结果回调给节点。

接口（见 docs/decisions/2026-10-24-第7步接口定稿-第五片.md）：
- stow  → action `control_msgs/action/FollowJointTrajectory`
          `/a200_0000/arm_0_joint_trajectory_controller/follow_joint_trajectory`
- release → action `control_msgs/action/GripperCommand`
          `/a200_0000/arm_0_gripper_controller/gripper_cmd`（position=0.0 即张开）
"""

from control_msgs.action import FollowJointTrajectory, GripperCommand
from rclpy.action import ActionClient
from trajectory_msgs.msg import JointTrajectoryPoint
import time
from moveit_msgs.msg import Constraints, JointConstraint
from moveit_msgs.srv import GetMotionPlan
from std_msgs.msg import Bool
from .place_plan import goal_constraints
import subprocess
import os
from pathlib import Path

ARM_JOINTS = ['arm_0_joint_%d' % i for i in range(1, 7)]


class ArmClient:
    def __init__(self, node, stow_action, gripper_action, on_stow, on_release):
        self.node = node
        self.on_stow, self.on_release = on_stow, on_release
        self.stow_client = ActionClient(node, FollowJointTrajectory, stow_action)
        self.gripper_client = ActionClient(node, GripperCommand, gripper_action)
        self.planner = node.create_client(GetMotionPlan, '/a200_0000/plan_kinematic_path')
        self.handle, self.cancel_requested = None, False
        self.deadline, self.callback = None, None
        self.after_move = None
        self.scene_process = None

    def stow(self, seconds=4.0):
        """Plan a collision-checked route to zero; never interpolate blindly through the table."""
        self._begin(self.on_stow, 240.)
        self.after_move = None
        if not self.planner.service_is_ready():
            self._finish(False, 'stow_planner_unavailable')
            return
        request = GetMotionPlan.Request()
        plan = request.motion_plan_request
        plan.group_name, plan.num_planning_attempts, plan.allowed_planning_time = 'arm', 3, 5.
        plan.start_state.is_diff = True
        plan.max_velocity_scaling_factor = plan.max_acceleration_scaling_factor = .15
        plan.goal_constraints = [Constraints(joint_constraints=[JointConstraint(
            joint_name=j, position=0., tolerance_above=.02, tolerance_below=.02, weight=1.)
            for j in ARM_JOINTS])]
        self.planner.call_async(request).add_done_callback(self._planned)

    def _planned(self, future):
        if self.cancel_requested or self.callback is None:
            self._finish(False, 'canceled')
            return
        try:
            response = future.result().motion_plan_response
            if response.error_code.val != 1 or not response.trajectory.joint_trajectory.points:
                self._finish(False, 'stow_plan_%d' % response.error_code.val)
                return
            goal = FollowJointTrajectory.Goal(trajectory=response.trajectory.joint_trajectory)
            duration = goal.trajectory.points[-1].time_from_start
            self.node.get_logger().info('Planned arm trajectory: %d points, %.2f sim seconds' %
                (len(goal.trajectory.points), duration.sec+duration.nanosec/1e9))
            self.stow_client.send_goal_async(goal).add_done_callback(self._stow_response)
        except Exception as error:
            self._finish(False, str(error))

    def release(self):
        self._begin(self.on_release, 240.)
        self.after_move = self._open
        run = Path(os.environ['Q2Q_STEP3_RUN'])
        loader = Path.home()/'ros2_ws/src/robot_moveit/scripts/scene_loader.py'
        with (run/'place_scene.log').open('w') as log:
            self.scene_process = subprocess.Popen(['python3', str(loader), '--world',
                str(run/'world.sdf'), '--skip-target'], stdout=log, stderr=subprocess.STDOUT)

    def _plan_place(self):
        request = GetMotionPlan.Request()
        plan = request.motion_plan_request
        plan.group_name, plan.num_planning_attempts, plan.allowed_planning_time = 'arm', 3, 5.
        plan.start_state.is_diff = True
        plan.max_velocity_scaling_factor = plan.max_acceleration_scaling_factor = .15
        plan.goal_constraints = goal_constraints()
        self.planner.call_async(request).add_done_callback(self._planned)

    def _open(self):
        self.node.suction_publisher.publish(Bool(data=False))
        goal = GripperCommand.Goal()
        goal.command.position = 0.0          # 0.0 = 张开（10-23 实测反直觉）
        goal.command.max_effort = 5.0        # 位置仿真不执行，仅接口兼容
        self.gripper_client.send_goal_async(goal).add_done_callback(self._release_response)

    def _stow_response(self, future):
        handle = future.result()
        if handle is None or not handle.accepted:
            self._finish(False, 'stow_goal_rejected')
            return
        self.handle = handle
        if self.cancel_requested:
            handle.cancel_goal_async()
        handle.get_result_async().add_done_callback(self._stow_result)

    def _release_response(self, future):
        handle = future.result()
        if handle is None or not handle.accepted:
            self._finish(False, 'release_goal_rejected')
            return
        self.handle = handle
        if self.cancel_requested:
            handle.cancel_goal_async()
        handle.get_result_async().add_done_callback(self._release_result)

    def _stow_result(self, future):
        wrapped = future.result()
        ok = wrapped.status == 4 and wrapped.result.error_code == 0
        if ok and self.after_move is not None:
            after, self.after_move = self.after_move, None
            self.handle = None
            after()
            return
        self._finish(ok, 'ok' if ok else 'stow_status_%d' % wrapped.status)

    def _release_result(self, future):
        status = future.result().status
        self._finish(status == 4, 'ok' if status == 4 else 'release_status_%d' % status)

    def _begin(self, callback, seconds):
        self.handle, self.cancel_requested = None, False
        self.callback, self.deadline = callback, time.monotonic()+seconds

    def _finish(self, ok, reason):
        callback, self.callback = self.callback, None
        self.deadline = None
        if callback is not None:
            callback(ok, reason)

    def cancel(self):
        self.cancel_requested = True
        if self.scene_process is not None:
            self.scene_process.terminate()
            self.scene_process = None
            self._finish(False, 'canceled')
        if self.handle is not None:
            self.handle.cancel_goal_async()

    def check_timeout(self):
        if self.scene_process is not None and self.scene_process.poll() is not None:
            code = self.scene_process.returncode
            self.scene_process = None
            if code == 0:
                self._plan_place()
            else:
                self._finish(False, 'place_scene_refresh_failed')
        if self.deadline is not None and time.monotonic() > self.deadline:
            self.cancel()
            self._finish(False, 'arm_timeout_stop_unconfirmed')
