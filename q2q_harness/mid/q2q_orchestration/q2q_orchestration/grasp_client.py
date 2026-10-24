"""抓取 action 的客户端封装：只管「发一次 / 取消一次 / 报结果」，不做状态决策。

和 nav_client 一个套路：异步 action 的将来回调最容易写错（重复发、取消竞态），
集中在一个文件里；节点只接收 `on_result(success, reason)`。
"""

from q2q_interfaces.action import Grasp
from rclpy.action import ActionClient


class GraspClient:
    def __init__(self, node, action, on_result, on_phase):
        self.node, self.on_result, self.on_phase = node, on_result, on_phase
        self.client = ActionClient(node, Grasp, action)
        self.handle = None

    def send(self, target_id, timeout_s):
        """发一次 goal。**不重发**：重试由状态机决定。"""
        goal = Grasp.Goal()
        goal.target_id, goal.timeout_s = target_id, float(timeout_s)
        self.client.send_goal_async(goal, feedback_callback=self._feedback).add_done_callback(
            self._response)
        return True

    def cancel(self):
        if self.handle is not None:
            self.handle.cancel_goal_async()
            return True
        return False

    def _feedback(self, feedback):
        self.on_phase(feedback.feedback.phase)

    def _response(self, future):
        handle = future.result()
        if handle is None or not handle.accepted:
            self.on_result(False, 'grasp_goal_rejected')
            return
        self.handle = handle
        handle.get_result_async().add_done_callback(self._result)

    def _result(self, future):
        result = future.result().result
        self.on_result(bool(result.success), str(result.reason))
