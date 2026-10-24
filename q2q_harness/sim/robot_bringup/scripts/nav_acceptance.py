#!/usr/bin/env python3
"""Nav2 单次验收：map 位姿、真实 action 终态、新鲜里程计与持续停稳。"""

import argparse
import json
import math
from pathlib import Path
import time

import rclpy
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry, OccupancyGrid
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from lifecycle_msgs.srv import GetState
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.time import Time
import tf2_ros

NS = '/a200_0000'
STATUS = {4: 'SUCCEEDED', 5: 'CANCELED', 6: 'ABORTED'}


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y),
                      1 - 2 * (q.y * q.y + q.z * q.z))


class Acceptance(Node):
    def __init__(self):
        super().__init__('nav_acceptance')
        self.client = ActionClient(self, NavigateToPose, NS + '/navigate_to_pose')
        self.lifecycle = self.create_client(GetState, NS + '/bt_navigator/get_state')
        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer, self)
        self.odom = None
        self.map_received = False
        self.create_subscription(OccupancyGrid, NS+'/map', self.on_map,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.odom_received = -math.inf
        self.max_linear = 0.0
        self.max_angular = 0.0
        self.trace = []
        self.last_trace = -math.inf
        self.last_report = time.monotonic()
        self.create_subscription(Odometry, NS + '/ground_truth/odom', self.on_odom, 20)

    def on_odom(self, msg):
        self.odom = msg
        self.odom_received = time.monotonic()
        v = msg.twist.twist
        self.max_linear = max(self.max_linear, math.hypot(v.linear.x, v.linear.y))
        self.max_angular = max(self.max_angular, abs(v.angular.z))

    def on_map(self, msg):
        self.map_received = len(msg.data) == msg.info.width*msg.info.height and len(msg.data) > 0

    def pose(self):
        transform = self.buffer.lookup_transform('map', 'base_footprint', Time())
        stamp = Time.from_msg(transform.header.stamp)
        if abs((self.get_clock().now() - stamp).nanoseconds) > 1_000_000_000:
            raise RuntimeError('map TF 超过 1 仿真秒未更新')
        p, q = transform.transform.translation, transform.transform.rotation
        return [p.x, p.y, yaw_of(q)]

    def fresh(self):
        if self.odom is None or time.monotonic() - self.odom_received > 2.0:
            return False
        stamp = Time.from_msg(self.odom.header.stamp)
        return abs((self.get_clock().now() - stamp).nanoseconds) < 1_000_000_000

    def tick(self):
        rclpy.spin_once(self, timeout_sec=0.05)
        if time.monotonic() - self.last_trace >= 0.2 and self.fresh():
            try:
                p = self.pose()
            except Exception:
                return
            physical = self.odom.pose.pose
            self.trace.append([self.get_clock().now().nanoseconds / 1e9, *p,
                               physical.position.x, physical.position.y, yaw_of(physical.orientation)])
            self.last_trace = time.monotonic()
            if time.monotonic() - self.last_report >= 5:
                print('MAP_POSE %.3f %.3f %.3f' % tuple(p), flush=True)
                self.last_report = time.monotonic()

    def wait(self, future, seconds):
        deadline = time.monotonic() + seconds
        while not future.done() and time.monotonic() < deadline:
            self.tick()
        if not future.done():
            raise TimeoutError('动作/服务等待超时 %.1f 秒' % seconds)
        return future.result()

    def ready(self):
        deadline = time.monotonic() + 60
        state_future = None
        state_sent = 0.0
        active = False
        while time.monotonic() < deadline:
            self.tick()
            if state_future is not None and not state_future.done() and time.monotonic() - state_sent > 2.0:
                state_future.cancel()     # 只重试只读 get_state，不重复发送运动目标。
                state_future = None
            if state_future is None and self.lifecycle.service_is_ready():
                state_future = self.lifecycle.call_async(GetState.Request())
                state_sent = time.monotonic()
            if state_future is not None and state_future.done():
                reply = state_future.result() if not state_future.exception() else None
                active = bool(reply and reply.current_state.id == 3)
                state_future = None
            try:
                self.pose()
                if active and self.fresh() and self.map_received and self.client.server_is_ready():
                    return
            except Exception:
                pass
        raise TimeoutError('60 秒内未同时获得 action server、map TF、新鲜 odom和OccupancyGrid')

    def stopped(self, timeout=30):
        started, quiet_wall, quiet_sim = time.monotonic(), None, None
        while time.monotonic() - started < timeout:
            self.tick()
            twist = self.odom.twist.twist if self.fresh() else None
            quiet = (twist is not None and math.hypot(twist.linear.x, twist.linear.y) < 0.02
                     and abs(twist.angular.z) < 0.05)
            if not quiet:
                quiet_wall, quiet_sim = None, None
                continue
            now_sim = self.get_clock().now().nanoseconds / 1e9
            if quiet_wall is None:
                quiet_wall, quiet_sim = time.monotonic(), now_sim
            if time.monotonic() - quiet_wall >= 0.5 and now_sim - quiet_sim >= 0.5:
                return time.monotonic() - started
        raise TimeoutError('30 秒内未满足新鲜 odom 持续停稳 0.5 仿真秒')

    def cancel(self, handle, result_future):
        start = time.monotonic()
        response = self.wait(handle.cancel_goal_async(), 15)
        if response.return_code != 0 or not response.goals_canceling:
            raise RuntimeError('服务器未接受取消：return_code=%s' % response.return_code)
        result = self.wait(result_future, 15)
        if result.status != GoalStatus.STATUS_CANCELED:
            raise RuntimeError('取消后终态不是 CANCELED：%s' % result.status)
        stop_seconds = self.stopped()
        return result, time.monotonic() - start, stop_seconds

    def run_goal(self, args, record):
        self.ready()
        record['initial_map_pose'] = self.pose()
        print('READY map TF + fresh odom + action server', flush=True)
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        x, y, yaw = args.goal
        goal.pose.pose.position.x, goal.pose.pose.position.y = x, y
        goal.pose.pose.orientation.z = math.sin(yaw / 2)
        goal.pose.pose.orientation.w = math.cos(yaw / 2)
        handle = self.wait(self.client.send_goal_async(goal), 20)
        if not handle.accepted:
            raise RuntimeError('目标被服务器拒绝')
        print('GOAL_ACCEPTED', flush=True)
        result_future = handle.get_result_async()
        try:
            if args.cancel_after:
                deadline = time.monotonic() + args.cancel_after
                while time.monotonic() < deadline and not result_future.done():
                    self.tick()
                if result_future.done():
                    raise RuntimeError('取消测试在发送取消之前已经结束')
                result, latency, hold = self.cancel(handle, result_future)
                record.update(cancel_accepted=True, cancel_to_stopped_wall_s=latency,
                              stop_verification_wall_s=hold)
                if latency > 3.0:
                    raise RuntimeError('取消至停稳验证超过 3 秒墙钟预算')
                if self.max_linear < 0.03 and self.max_angular < 0.1:
                    raise RuntimeError('取消前未观测到运动，不能算有效取消测试')
            else:
                result = self.wait(result_future, args.timeout)
                record['status'] = STATUS.get(result.status, str(result.status))
                record['stop_verification_wall_s'] = self.stopped()
                if result.status != GoalStatus.STATUS_SUCCEEDED:
                    raise RuntimeError('导航终态 %s' % STATUS.get(result.status, result.status))
            record['status'] = STATUS.get(result.status, str(result.status))
        except Exception:
            if not result_future.done():
                self.cancel(handle, result_future)
            raise
        record['final_map_pose'] = self.pose()
        physical = self.odom.pose.pose
        twist = self.odom.twist.twist
        record.update(final_linear_mps=math.hypot(twist.linear.x, twist.linear.y),
                      final_angular_rps=abs(twist.angular.z), stop_hold_sim_s=0.5,
                      fresh_odometry=self.fresh())
        record['final_physical_pose'] = [physical.position.x, physical.position.y,
                                         yaw_of(physical.orientation)]
        px, py, heading = record['final_map_pose']
        record['xy_error_m'] = math.hypot(px - x, py - y)
        record['yaw_error_rad'] = abs(math.atan2(math.sin(heading - yaw), math.cos(heading - yaw)))
        # 本场景 map 与 world 的原点约定相同，直接核对仿真实体确实到位。
        record['physical_xy_error_m'] = math.hypot(physical.position.x - x, physical.position.y - y)
        physical_yaw = yaw_of(physical.orientation)
        record['physical_yaw_error_rad'] = abs(math.atan2(math.sin(physical_yaw - yaw), math.cos(physical_yaw - yaw)))
        if not args.cancel_after and (record['xy_error_m'] > 0.25 or record['yaw_error_rad'] > 0.25):
            raise RuntimeError('map 位姿误差超出 xy/yaw 验收阈值')
        if not args.cancel_after and (record['physical_xy_error_m'] > 0.25 or record['physical_yaw_error_rad'] > 0.25):
            raise RuntimeError('Gazebo 物理实体位姿误差超出验收阈值')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--goal', nargs=3, type=float, required=True)
    parser.add_argument('--cancel-after', type=float, default=0)
    parser.add_argument('--timeout', type=float, default=300)
    parser.add_argument('--output', required=True)
    args, _ = parser.parse_known_args()
    rclpy.init()
    node = Acceptance()
    record = {'goal': args.goal, 'passed': False, 'cancel_test': bool(args.cancel_after),
              'odometry_source': 'Gazebo physical ground truth', 'localization': 'ground_truth_identity_map',
              'trace_columns': ['sim_s', 'map_x', 'map_y', 'map_yaw', 'physical_x', 'physical_y', 'physical_yaw']}
    started = time.monotonic()
    try:
        node.run_goal(args, record)
        record['passed'] = True
    except Exception as exc:
        record['error'] = str(exc)
    finally:
        record.update(wall_seconds=time.monotonic() - started,
                      max_linear_mps=node.max_linear, max_angular_rps=node.max_angular,
                      trace=node.trace)
        output = Path(args.output).expanduser()
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(record, indent=2, allow_nan=False), encoding='utf-8')
        print(json.dumps({k: v for k, v in record.items() if k != 'trace'}), flush=True)
        node.destroy_node()
        rclpy.shutdown()
    return 0 if record['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
