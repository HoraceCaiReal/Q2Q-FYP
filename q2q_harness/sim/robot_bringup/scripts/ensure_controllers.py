#!/usr/bin/env python3
"""ensure_controllers.py —— 启动自检：让第 3 步要用的控制器都进 active（幂等）

**为什么需要这个文件**（2026-10-21 实测，不是猜的）：

  1. 生成机器人时，`gz_ros2_control` 插件在 Gazebo 进程里建 controller_manager；
     同一时刻 Gazebo 还要初始化激光的 ogre2 渲染（软渲染，llvmpipe）。这会把 CM 的服务处理堵住。
  2. Clearpath 的 spawner 调 `load_controller`，默认 `--service-call-timeout 10`：
        Failed getting a result from calling /a200_0000/controller_manager/load_controller in 10.0
  3. 它重试；这次请求到的时候控制器**已经装好了** → CM 回 "already loaded" → spawner 判致命：
        FATAL Failed loading controller joint_state_broadcaster
  4. 于是 joint_state_broadcaster 停在 unconfigured：没有 /joint_states、没有 map 帧，
     slam_toolbox 和 Nav2 全线起不来。

  spawner 本身不是幂等的（已经 active 时再跑会报 "Failed to configure controller"），
  所以不能无脑重跑。这个脚本先查状态，**只补缺的那几个**：未加载就 load，
  未配置就 configure，未激活就 switch_controllers。全部 active 时什么都不做，直接退出。

调用方：bringup.launch.py 在 spawn 窗口之后（45 s）执行一次。
退出码：0 = 4 个都 active；1 = 仍有缺失（日志里会点名是哪几个）。
"""

import argparse
import sys
import time

import rclpy
from builtin_interfaces.msg import Duration
from controller_manager_msgs.srv import (ConfigureController, ListControllers,
                                         LoadController, SwitchController)
from rclpy.node import Node

REQUIRED = [
    'joint_state_broadcaster',
    'platform_velocity_controller',
    'arm_0_joint_trajectory_controller',
    'arm_0_gripper_controller',
]
SERVICE_TIMEOUT = 15.0
ATTEMPTS = 8
RETRY_PAUSE = 8.0


class ControllerFixer(Node):

    def __init__(self, namespace):
        super().__init__('ensure_controllers')
        base = '/%s/controller_manager' % namespace.strip('/') if namespace else '/controller_manager'
        # 注意：不能叫 self.clients —— rclpy 的 Node 已有只读属性 clients，
        # 赋值会抛 AttributeError: can't set attribute 'clients'（2026-10-21 踩过一次）
        self.srv = {
            'list': self.create_client(ListControllers, base + '/list_controllers'),
            'load': self.create_client(LoadController, base + '/load_controller'),
            'configure': self.create_client(ConfigureController, base + '/configure_controller'),
            'switch': self.create_client(SwitchController, base + '/switch_controller'),   # 注意是单数！
        }

    def call(self, key, request):
        client = self.srv[key]
        if not client.wait_for_service(timeout_sec=SERVICE_TIMEOUT):
            self.get_logger().error('等不到 %s；下一步该查：仿真是否起来、命名空间是否写对'
                                    % client.srv_name)
            return None
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=SERVICE_TIMEOUT)
        return future.result()

    def states(self):
        response = self.call('list', ListControllers.Request())
        return {c.name: c.state for c in response.controller} if response else {}

    def load(self, name):
        request = LoadController.Request()
        request.name = name
        response = self.call('load', request)
        # ok=False 且提示 already loaded 是正常情况（别的 spawner 抢先装了）
        return response is not None

    def configure(self, name):
        request = ConfigureController.Request()
        request.name = name
        response = self.call('configure', request)
        return response is not None and response.ok

    def activate(self, name):
        request = SwitchController.Request()
        request.activate_controllers = [name]
        request.deactivate_controllers = []
        request.strictness = SwitchController.Request.STRICT
        request.activate_asap = False
        # 坑：timeout 是 builtin_interfaces/Duration，不是 float。
        # 赋 30.0 会抛 AttributeError（2026-10-21 踩过）；0 表示无限等，不能用。
        request.timeout = Duration(sec=30, nanosec=0)
        response = self.call('switch', request)
        return response is not None and response.ok

    def ensure_one(self, name, state):
        """缺什么补什么。任何一步失败都不要抛异常 —— 下一轮会重试。"""
        try:
            if state is None:
                self.load(name)
                state = self.states().get(name)
            if state == 'unconfigured':
                self.configure(name)
                state = self.states().get(name)
            if state == 'inactive':
                self.activate(name)
                state = self.states().get(name)
        except Exception as error:      # noqa: BLE001 —— 这里就是要吞掉，不能让自检自己挂掉
            self.get_logger().error('%s 补的过程出错：%s' % (name, error))
            return False
        return state == 'active'

    def run(self):
        for attempt in range(1, ATTEMPTS + 1):
            states = self.states()
            if not states:
                self.get_logger().error('拿不到控制器列表（第 %d 次）；%0.f s 后重试'
                                        % (attempt, RETRY_PAUSE))
                time.sleep(RETRY_PAUSE)
                continue
            missing = [n for n in REQUIRED if n not in states or states[n] != 'active']
            if not missing:
                self.get_logger().info('4 个控制器全部 active（第 %d 次检查）' % attempt)
                return True
            self.get_logger().warn('第 %d 次检查：这 %d 个不在 active：%s —— 现在补'
                                   % (attempt, len(missing), ', '.join(missing)))
            for name in missing:
                if not self.ensure_one(name, states.get(name)):
                    self.get_logger().error('%s 这一轮没补上' % name)
            if attempt < ATTEMPTS:
                time.sleep(RETRY_PAUSE)
        final = self.states()
        broken = [n for n in REQUIRED if final.get(n) != 'active']
        if broken:
            self.get_logger().error(
                '仍有控制器没起来：%s；下一步该查：ros2 control list_controllers -c /<ns>/controller_manager，'
                '再看 /tmp 里的 launch 日志有没有 spawner FATAL' % ', '.join(broken))
            return False
        self.get_logger().info('4 个控制器全部 active')
        return True


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--namespace', default='a200_0000')
    args = parser.parse_args()
    rclpy.init()
    node = ControllerFixer(args.namespace)
    try:
        return 0 if node.run() else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
