#!/usr/bin/env python3
"""state_recorder：第 7 步第一片 —— 把 TaskState 逐条落成 JSONL。

为什么单独一个包：记录是独立职责（记录层），不能挂在调度节点里——
调度崩了，记录还得在；记录写盘失败，也不许静默丢样本。

落盘规则（接口定稿 I4）：
- 路径 `<record_root>/<episode_id>/task_state.jsonl`，一行一条样本；
- 每行 `flush`，`fsync` 按 `fsync_period` 秒做一次——既避免静默丢太多，又不把 10 Hz 写成磁盘抖动；
- 打不开/写失败 → 打 FATAL 并退出（**不允许静默丢记录**）；
- 中途收到**不同 episode_id** 的样本 → 拒绝并退出：把两次运行混进一个文件等于毁证据。
"""

import json
import os
import time
from pathlib import Path

import rclpy
from q2q_interfaces.msg import TaskState
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException

STATE_TOPIC = '/q2q/mid/task_state'
FIELDS = ('episode_id', 'task_id', 'generation', 'state', 'base_fresh', 'base_stopped',
          'base_command_locked', 'arm_stowed', 'arm_executing', 'base_x', 'base_y', 'base_yaw',
          'base_speed', 'base_yaw_rate', 'holding_object', 'blocked_by',
          'nav_goal_id', 'nav_status', 'distance_remaining', 'last_request_id', 'late_events')


class Recorder(Node):
    def __init__(self):
        super().__init__('state_recorder')
        self.declare_parameter('record_root', str(Path.home() / '.q2q/records'))
        self.declare_parameter('fsync_period', 1.0)
        self.root = Path(self.get_parameter('record_root').value)
        self.fsync_period = float(self.get_parameter('fsync_period').value)
        if self.fsync_period <= 0:
            raise SystemExit('fsync_period 必须为正；下一步该查：启动参数')
        self.file, self.episode, self.last_sync, self.count = None, None, time.monotonic(), 0
        self.create_subscription(TaskState, STATE_TOPIC, self.on_state, 10)
        self.get_logger().info('state_recorder 就绪：写 %s/<episode>/task_state.jsonl' % self.root)

    def open_for(self, episode):
        directory = self.root / episode
        directory.mkdir(parents=True, exist_ok=True)
        path = directory / 'task_state.jsonl'
        self.file = path.open('a', encoding='utf-8')
        self.episode = episode
        self.get_logger().info('记录开始：%s' % path)

    def on_state(self, msg):
        if self.episode is None:
            self.open_for(msg.episode_id or 'unknown')
        elif msg.episode_id != self.episode:
            raise SystemExit('样本的 episode_id=%s 与已打开的 %s 不一致；下一步该查：是否两个 episode 同时发布'
                             % (msg.episode_id, self.episode))
        row = {name: getattr(msg, name) for name in FIELDS}
        row['stamp_ns'] = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        row['wall_time'] = time.time()
        self.file.write(json.dumps(row, ensure_ascii=False) + '\n')
        self.file.flush()
        self.count += 1
        if time.monotonic() - self.last_sync >= self.fsync_period:
            os.fsync(self.file.fileno())
            self.last_sync = time.monotonic()

    def close(self):
        if self.file is not None:
            os.fsync(self.file.fileno())
            self.file.close()
            self.get_logger().info('记录结束：共 %d 条' % self.count)


def main():
    rclpy.init()
    node = Recorder()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
