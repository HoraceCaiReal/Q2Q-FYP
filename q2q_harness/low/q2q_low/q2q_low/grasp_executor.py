#!/usr/bin/env python3
"""grasp_executor：把「执行一次抓取流水线」做成 action（第 7 步第四片）。

实现选择（临时但有边界）：内部调用已经验证过的 `robot_bringup/scripts/grasp_run.py` 子进程，
把它自带的 P1–P6 门禁与 JSON 结果翻译成 action 结果。这样第 7 步先跑通整链，
而第 6 步那套证据链不用重写一遍。**后续把逻辑内联进来时，action 接口不变。**

两条硬约束：
1. 超时/取消都要**真的杀掉子进程**，不允许残留（否则下一轮会在脏世界里跑）；
2. 结果必须来自流水线的 JSON，不允许这里自己编 success。
"""

import json
import os
import subprocess
import time
import threading
from pathlib import Path

import rclpy
from q2q_interfaces.action import Grasp
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import SingleThreadedExecutor, ExternalShutdownException
from rclpy.clock import Clock, ClockType
from rclpy.task import Future
from rclpy.node import Node

SCRIPTS = Path.home() / 'ros2_ws/src/robot_bringup/scripts'
RUNS = Path.home() / '.q2q/runs'


class GraspExecutor(Node):
    def __init__(self):
        super().__init__('grasp_executor')
        self.declare_parameter('grasp_timeout_s', 240.0)
        self.declare_parameter('output_dir', str(RUNS))
        self.timeout_default = float(self.get_parameter('grasp_timeout_s').value)
        self.output_dir = Path(self.get_parameter('output_dir').value)
        self.declare_parameter('use_suction', False)
        self.declare_parameter('world_sdf', '')
        self.child = None
        self.busy = False
        self.faulted = False
        self.goal_lock = threading.Lock()
        self.pulse = None
        self.create_timer(.2, self.wake, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.server = ActionServer(
            self, Grasp, '/q2q/low/grasp', execute_callback=self.execute,
            goal_callback=self.accept_goal,
            cancel_callback=lambda goal: CancelResponse.ACCEPT,
            callback_group=ReentrantCallbackGroup())
        self.get_logger().info('grasp_executor 就绪：适配 grasp_run.py，结果以它的 JSON 为准')

    def accept_goal(self, goal):
        with self.goal_lock:
            if self.busy or self.faulted or goal.target_id != 'pick_bin':
                return GoalResponse.REJECT
            self.busy = True
            return GoalResponse.ACCEPT

    def wake(self):
        if self.pulse is not None and not self.pulse.done():
            self.pulse.set_result(True)

    async def execute(self, handle):
        try:
            return await self.run_pipeline(handle)
        except Exception as error:
            self.kill_child()
            handle.abort()
            return Grasp.Result(success=False, reason=str(error), rise_mm=0.0)
        finally:
            with self.goal_lock:
                self.busy = False

    async def run_pipeline(self, handle):
        requested = float(handle.request.timeout_s)
        timeout = requested if requested > 0 else self.timeout_default
        # 文件名用墙钟：仿真钟在暂停/重置时会重复，撞名会覆盖上一轮证据
        stamp = '%d_%d' % (int(time.time()), self.get_clock().now().nanoseconds // 1000000)
        output = self.output_dir / ('grasp_action_%s.json' % stamp)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        command = ['python3', str(SCRIPTS / 'grasp_run.py'), '--output', str(output)]
        command += self.fixture_args()
        if self.get_parameter('use_suction').value:
            command.append('--suction')          # 运动学搬运（不是夹持）
        world = self.get_parameter('world_sdf').value
        command = ['python3', str(Path(__file__).with_name('grasp_job.py')), world] + command
        feedback = Grasp.Feedback()
        feedback.phase = 'start'
        handle.publish_feedback(feedback)
        result = Grasp.Result()
        result.success, result.reason, result.rise_mm = False, '', 0.0
        # Keep the actual failing phase and traceback, not just the final boolean.
        with output.with_suffix('.log').open('w') as log:
            self.child = subprocess.Popen(command, stdout=log,
                                          stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL,
                                          start_new_session=True)
        deadline = time.monotonic() + timeout
        while True:
            if self.child.poll() is not None:
                break
            # 每 0.2 s 查一次取消：**不能**用 child.wait(timeout) 阻塞着等，
            # 那样取消请求要等流水线自己跑完才被看到，等于取消无效（10-24 实测过）。
            if handle.is_cancel_requested:
                stopped = self.kill_child()
                confirmed = stopped and output.exists() and json.loads(
                    output.read_text()).get('motion_stop_confirmed', False)
                result.reason = 'canceled' if confirmed else 'grasp_stop_unconfirmed'
                if confirmed:
                    handle.canceled()
                else:
                    self.faulted = True
                    handle.abort()
                return result
            if time.monotonic() > deadline:
                self.kill_child()
                result.reason = 'timeout'
                handle.abort()
                return result
            self.pulse = Future()
            await self.pulse
        code = self.child.returncode
        self.child = None
        result.success, result.reason, result.rise_mm = self.read_result(output, code)
        if result.success:
            handle.succeed()
        else:
            handle.abort()
        return result

    def fixture_args(self):
        """从本轮世界快照读目标尺寸与中心 → grasp_run 需要的参数。

        为什么不让调用方传：第 7 步的编排只该关心「抓」，不该关心 fixture 尺寸；
        而 grasp_run 的 P1 门禁要求 box-center 与世界一致，写死会把 50 mm 世界判成脏基线。
        """
        import xml.etree.ElementTree as ET
        path = self.get_parameter('world_sdf').value
        if not path:
            run = os.environ.get('Q2Q_STEP3_RUN', '')
            path = str(Path(run) / 'world.sdf') if run else ''
        if not path or not Path(path).exists():
            return []
        model = ET.parse(path).getroot().find('world/model')
        world = ET.parse(path).getroot().find('world')
        if world is None:
            return []
        for candidate in world.findall('model'):
            if candidate.get('name') == 'pick_bin':
                model = candidate
                break
        if model is None or model.get('name') != 'pick_bin':
            return []
        center = model.findtext('pose').split()[:3]
        size = float(model.findtext('link/collision/geometry/box/size').split()[0])
        if abs(size - .05) < 1e-6:
            # Kinematic carry does not need contact force. Use a clear-of-object jaw
            # animation, rather than deliberately stalling/faulting a physical controller.
            close = '.20' if self.get_parameter('use_suction').value else '.70'
            return ['--box-center'] + center + ['--close-grip', close, '--tip-past', '-.010']
        if abs(size - .08) < 1e-6:
            return ['--box-center'] + center + ['--close-grip', '.15', '--tip-past', '-.015']
        return ['--box-center'] + center

    def read_result(self, output, code):
        """只认流水线自己的 JSON：这里不猜成功。"""
        if not output.exists():
            return False, 'no_result_json_exit_%d' % code, 0.0
        report = json.loads(output.read_text(encoding='utf-8'))
        if code == 0 and report.get('passed'):
            return True, 'ok', float(report.get('P6', {}).get('rise_mm', 0.0))
        return False, str(report.get('error', 'pipeline_failed_exit_%d' % code)), 0.0

    def kill_child(self):
        if self.child is None:
            return True
        try:
            os.killpg(os.getpgid(self.child.pid), 15)
            self.child.wait(timeout=20)
            return True
        except subprocess.TimeoutExpired:
            os.killpg(os.getpgid(self.child.pid), 9)
            self.child.wait(timeout=5)
            return False  # Process death is not confirmation that its controller stopped.
        except ProcessLookupError:
            return False


def main():
    rclpy.init()
    node = GraspExecutor()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.kill_child()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
