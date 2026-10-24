#!/usr/bin/env python3
"""第7步串行调度：导航、对接、锁底盘、抓取、收臂、出站、返回、投放。

接口和当前验收见 docs/evaluations/2026-10-24-第7步整链修复与验收.md。
运动学携带仅是明确标注的仿真近似，不能作为真实抓取证据。
"""

import math
import time

import rclpy
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from q2q_interfaces.msg import TaskCommand, TaskState
from rclpy.node import Node
from sensor_msgs.msg import JointState
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException

from . import machine as sm
from .arm_client import ArmClient
from .dock import Docker
from .grasp_client import GraspClient
from .measure import WorldMeasure, bounded_param
from .nav_client import NavClient
from .ready_gate import ReadyGate
from .state_publish import build

ODOM, JOINTS = '/a200_0000/ground_truth/odom', '/a200_0000/platform/joint_states'
STATE_TOPIC, COMMAND_TOPIC = '/q2q/mid/task_state', '/q2q/mid/task_command'
PARAMS = (('episode_id', ''), ('task_id', 'slice3'), ('freshness_window', 0.3),
          ('stopped_window', 0.5), ('v_eps', 0.01), ('w_eps', 0.02), ('stowed_rad', 0.25),
          ('publish_rate', 10.0), ('nav_action', '/a200_0000/navigate_to_pose'),
          ('nav_timeout_s', 90.0), ('corridor', [0.20, 1.20, -0.60, 0.60]),
          ('dock_window', [0.86, 0.90, 0.02, 0.04]), ('dock_speed', 0.08),
          ('dock_timeout_s', 20.0), ('dock_rate', 20.0),
          ('lock_topic', '/a200_0000/platform/emergency_stop'),
          ('cmd_topic', '/a200_0000/cmd_vel'), ('grasp_action', '/q2q/low/grasp'),
          ('grasp_timeout_s', 240.0), ('target_id', 'pick_bin'),
          ('stow_action', '/a200_0000/arm_0_joint_trajectory_controller/follow_joint_trajectory'),
          ('gripper_action', '/a200_0000/arm_0_gripper_controller/gripper_cmd'),
          ('place_pose', [0.10, 0.0, 0.0]), ('arrive_x', 0.80),
          ('nav_recover_band', 0.10), ('nav_ready_timeout_s', 45.0), ('picking_stale_s', 10.0))


class Orchestration(Node):
    def __init__(self):
        super().__init__('orchestration_node')
        for name, default in PARAMS:
            self.declare_parameter(name, default)
        self.episode = self.get_parameter('episode_id').value
        if not self.episode:
            raise SystemExit('episode_id 为空；下一步该查：启动时传 -p episode_id:=<编号>')
        rate = bounded_param(self, 'publish_rate', 0.0, 100.0)
        self.nav_timeout = bounded_param(self, 'nav_timeout_s', 0.0, 600.0)
        corridor = list(self.get_parameter('corridor').value)
        if len(corridor) != 4 or corridor[0] >= corridor[1] or corridor[2] >= corridor[3]:
            raise SystemExit('corridor 必须是递增的 [xmin,xmax,ymin,ymax]；下一步该查：启动参数')
        self.measure = WorldMeasure(bounded_param(self, 'freshness_window', 0.0, 5.0),
                                    bounded_param(self, 'stopped_window', 0.0, 5.0),
                                    self.get_parameter('v_eps').value,
                                    self.get_parameter('w_eps').value,
                                    bounded_param(self, 'stowed_rad', 0.0, math.pi))
        window = list(self.get_parameter('dock_window').value)
        if len(window) != 4 or window[0] >= window[1] or window[2] <= 0 or window[3] <= 0:
            raise SystemExit('dock_window 必须是 [xmin,xmax,|y|,|yaw|]；下一步该查：启动参数')
        self.machine = sm.Machine(tuple(corridor), tuple(window))
        self.machine.dock_timeout = bounded_param(self, 'dock_timeout_s', 0.0, 120.0)
        # 真值接管的两条线：过 arrive_x 就自己叫停；abort 后离工位不足 recover_band 就还能救
        self.machine.arrive_x = bounded_param(self, 'arrive_x', 0.20, 1.20)
        self.machine.recover_band = bounded_param(self, 'nav_recover_band', 0.0, 0.50)
        self.machine.picking_stale = bounded_param(self, 'picking_stale_s', 0.5, 900.0)
        self.gate = ReadyGate(bounded_param(self, 'nav_ready_timeout_s', 0.0, 600.0))
        dock_rate = bounded_param(self, 'dock_rate', 0.0, 100.0)
        self.docker = Docker(self, self.get_parameter('cmd_topic').value, tuple(window),
                             bounded_param(self, 'dock_speed', 0.0, 0.20))
        self.lock_publisher = self.create_publisher(Bool, self.get_parameter('lock_topic').value, 1)
        self.suction_publisher = self.create_publisher(Bool, '/q2q/low/suction', 1)
        self.locked = False
        self.nav = NavClient(self, self.get_parameter('nav_action').value,
                             self.on_nav_status, self.on_distance, self.on_goal)
        self.grasp = GraspClient(self, self.get_parameter('grasp_action').value,
                                 self.on_grasp_result, self.on_grasp_phase)
        self.arm = ArmClient(self, self.get_parameter('stow_action').value,
                             self.get_parameter('gripper_action').value,
                             self.on_stow, self.on_release)
        place = list(self.get_parameter('place_pose').value)
        if len(place) != 3:
            raise SystemExit('place_pose 必须是 [x,y,yaw]；下一步该查：启动参数')
        self.machine.place = tuple(place)
        self.create_subscription(Odometry, ODOM, lambda m: self.measure.on_odom(m, time.monotonic()), 10)
        self.create_subscription(JointState, JOINTS,
                                 lambda m: self.measure.on_joints(m, time.monotonic()), 10)
        self.create_subscription(TaskCommand, COMMAND_TOPIC, self.on_command, 10)
        self.publisher = self.create_publisher(TaskState, STATE_TOPIC, 1)
        steady = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(1.0 / rate, self.tick, clock=steady)
        self.create_timer(1.0 / dock_rate, self.dock_tick, clock=steady)
        self.get_logger().info('orchestration_node 就绪：episode=%s（串行移动取放）' % self.episode)

    def missing_ready(self):
        """START 的三个就绪条件；缺一个都不该发出 goal（缺了会白烧导航预算或直接被状态机拒绝）。"""
        conds = self.measure.conditions(time.monotonic())
        checks = (('nav_active', self.nav.ready(time.monotonic())),
                  ('odom_fresh', conds['fresh']),
                  ('arm_stowed', conds['stowed']))
        return [name for name, ok in checks if not ok]

    def nav_ready(self):
        return not self.missing_ready()

    def on_command(self, msg):
        if msg.command == TaskCommand.START:
            if self.machine.state not in (sm.IDLE, sm.FAILED, sm.DONE):
                self.get_logger().warn('START ignored: task already active')
                return
            allowed = self.gate.offer(msg, time.monotonic(), self.nav_ready())
            if allowed is None:
                self.get_logger().warn('START 先暂存：还缺 %s（不等就白烧导航超时）'
                                       % ','.join(self.missing_ready()))
                self.publish()
                return
            timeout = msg.timeout_s if msg.timeout_s > 0 else self.nav_timeout
            actions = self.machine.start(time.monotonic(), msg.request_id,
                                         (msg.target_x, msg.target_y, msg.target_yaw),
                                         timeout, self.measure.conditions(time.monotonic()))
            if not actions and self.machine.state == sm.IDLE:
                # 状态机拒绝了：**必须喊出来**。上一轮它静默拒绝，运行器还在傻等
                self.get_logger().error('START 被状态机拒绝：%s' % self.machine.reason)
        elif msg.command == TaskCommand.CANCEL:
            self.gate.pending = None
            actions = self.machine.cancel()
        else:
            return
        self.run_actions(actions)
        self.publish()

    def run_actions(self, actions):
        for action in actions:
            if action[0] == 'send_goal':
                self.machine.started = time.monotonic()
                self.nav.send(action[1], self.get_clock().now().to_msg())
            elif action[0] == 'cancel_goal':
                self.nav.cancel()
                self.get_logger().warn('已请求取消导航 goal（只发一次）')
            elif action[0] == 'stop_base':
                self.docker._publish(0.0)
                self.docker.active = False
            elif action[0] == 'lock':
                self.set_lock(True)
            elif action[0] == 'unlock':
                self.set_lock(False)
            elif action[0] == 'grasp':
                timeout = bounded_param(self, 'grasp_timeout_s', 0.0, 900.0)
                self.grasp.send(self.get_parameter('target_id').value, timeout)
                self.get_logger().warn('已发出抓取 goal（只发一次）')
            elif action[0] == 'cancel_grasp':
                self.grasp.cancel()
                self.get_logger().warn('已请求取消抓取 goal')
            elif action[0] == 'stow':
                self.arm.stow()
                self.get_logger().warn('已发出收臂轨迹（只发一次）')
            elif action[0] == 'cancel_arm':
                self.arm.cancel()
            elif action[0] == 'release':
                # 吸附模式：先断开吸附再张爪（顺序反了会把物体带着甩出去）
                self.arm.release()
                self.get_logger().warn('先规划到车外投放姿态，再 detach 和张爪')

    def set_lock(self, locked):
        """只发一次；这个锁 timeout=0，不会自己过期，所以必须记着谁持有它。"""
        if locked == self.locked:
            return
        self.locked = locked
        self.lock_publisher.publish(Bool(data=locked))
        self.get_logger().warn('底盘控制权锁：%s' % ('已锁定' if locked else '已释放'))

    def dock_tick(self):
        if self.machine.state == sm.UNDOCKING:
            conds = self.measure.conditions(time.monotonic())
            allowed = conds['fresh'] and conds['stowed'] and conds['x'] > .64
            self.docker.active = True
            self.docker._publish(-.08 if allowed else 0.)
            return
        self.docker.update(self.machine.state == sm.DOCKING, self.measure.conditions(time.monotonic()))

    def on_stow(self, ok, reason):
        if reason == 'arm_timeout_stop_unconfirmed':
            self.run_actions(self.machine.fail(reason))  # Keep base locked until stop is confirmed.
            return
        if self.machine.state == sm.CANCELING and self.machine.reason == 'cancel_arm':
            if reason != 'arm_timeout_stop_unconfirmed':
                self.run_actions(self.machine.arm_canceled())
            return
        self.run_actions(self.machine.stow_result(ok, reason))
        if self.machine.state == sm.UNDOCKING:
            self.machine.undock_started = time.monotonic()
        self.get_logger().warn('收臂结果 ok=%s reason=%s → %s' % (ok, reason, self.machine.state))
        self.publish()

    def on_release(self, ok, reason):
        if reason == 'arm_timeout_stop_unconfirmed':
            self.run_actions(self.machine.fail(reason))
            return
        if self.machine.state == sm.CANCELING and self.machine.reason == 'cancel_arm':
            if reason != 'arm_timeout_stop_unconfirmed':
                self.run_actions(self.machine.arm_canceled())
            return
        self.run_actions(self.machine.release_result(ok, reason))
        self.get_logger().warn('松爪结果 ok=%s reason=%s → %s' % (ok, reason, self.machine.state))
        self.publish()

    def on_grasp_phase(self, phase):
        self.get_logger().info('抓取阶段：%s' % phase)

    def on_grasp_result(self, success, reason):
        self.run_actions(self.machine.grasp_result(success, reason))
        self.get_logger().warn('抓取结果 success=%s reason=%s → 状态=%s（%s）'
                               % (success, reason, self.machine.state, self.machine.reason))
        self.publish()

    def on_goal(self, goal_id):
        self.machine.active_goal = goal_id

    def on_distance(self, distance):
        self.machine.distance = distance

    def on_nav_status(self, status, goal_id):
        self.run_actions(self.machine.nav_result(status, goal_id))
        self.get_logger().info('导航终态=%s → 状态=%s（%s）'
                               % (status, self.machine.state, self.machine.reason))
        self.publish()

    def tick(self):
        now = time.monotonic()
        self.arm.check_timeout()
        allowed, expired = self.gate.poll(now, self.nav_ready())
        if allowed is not None:
            self.get_logger().warn('导航服务已就绪：放行暂存的 START')
            self.on_command(allowed)
        elif expired:
            self.get_logger().error('等就绪超时（%.0f s）：还缺 %s；下一步该查：Nav2 是否起来 / 真值桥是否在发'
                                    % (self.gate.timeout_s, ','.join(self.missing_ready())))
            self.run_actions(self.machine.fail('not_ready_after_%.0fs' % self.gate.timeout_s))
        self.run_actions(self.machine.tick(now, self.measure.conditions(now)))
        self.publish()

    def publish(self):
        stamp = self.get_clock().now().to_msg()
        conds = self.measure.conditions(time.monotonic())
        message = build(self.machine, conds, self.episode,
                         self.get_parameter('task_id').value, stamp)
        message.base_command_locked = self.locked          # 第三片起：真实报告，不再是占位
        if self.gate.pending is not None:
            message.blocked_by = 'waiting_nav_ready'       # 暂存中：让操作者看得见为什么还没动
        self.publisher.publish(message)


def main():
    rclpy.init()
    node = Orchestration()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node.locked and 'unconfirmed' not in node.machine.reason:
            node.set_lock(False)          # 兜底解锁：timeout=0 的锁不会自己过期
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
