"""导航 action 的客户端封装：只管「发一次 / 取消一次 / 报终态」，不做状态决策。

抽出来的原因有两个：
1. 异步 action 的将来回调最容易写错（重复发 goal、取消前的竞态），集中在一个文件里好读；
2. 节点文件一膨胀就过不了行数上限。

约定：`on_status(status, goal_id)` 由节点接收，status ∈ SUCCEEDED/CANCELED/ABORTED/REJECTED；
`goal_id` 用于丢弃「不属于当前目标」的迟到结果。
"""

import math

from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from lifecycle_msgs.srv import GetState

STATUS = {4: 'SUCCEEDED', 5: 'CANCELED', 6: 'ABORTED'}


class NavClient:
    def __init__(self, node, action, on_status, on_distance, on_goal):
        self.node, self.on_status, self.on_distance, self.on_goal = node, on_status, on_distance, on_goal
        self.client = ActionClient(node, NavigateToPose, action)
        self.handle, self.pending = None, ''
        self.cancel_pending = False
        self.lifecycle = node.create_client(GetState, action.rsplit('/', 1)[0]+'/bt_navigator/get_state')
        self.active = False
        self.state_future, self.checked_at = None, 0.

    def ready(self, now):
        if self.state_future is not None and self.state_future.done():
            try:
                self.active = self.state_future.result().current_state.id == 3
            except Exception:
                self.active = False
            self.state_future = None
        if self.state_future is not None and now-self.checked_at > 5.:
            self.state_future.cancel()
            self.state_future, self.active = None, False
        if self.state_future is None and now-self.checked_at > 1. and self.lifecycle.service_is_ready():
            self.checked_at = now
            self.state_future = self.lifecycle.call_async(GetState.Request())
        return self.client.server_is_ready() and self.active

    def send(self, target, stamp):
        """发一次 goal。**不做重发**：重试必须由状态机用新 request_id 决定。"""
        x, y, yaw = target
        self.handle, self.pending, self.cancel_pending = None, '', False
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id, goal.pose.header.stamp = 'map', stamp
        goal.pose.pose.position.x, goal.pose.pose.position.y = x, y
        goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        self.client.send_goal_async(goal, feedback_callback=self._feedback).add_done_callback(
            self._response)

    def cancel(self):
        self.cancel_pending = True
        if self.handle is not None:
            self.handle.cancel_goal_async()

    def _feedback(self, feedback):
        self.on_distance(float(feedback.feedback.distance_remaining))

    def _response(self, future):
        handle = future.result()
        if handle is None or not handle.accepted:
            self.on_status('REJECTED', '')
            return
        self.handle = handle
        self.pending = handle.goal_id.uuid.tobytes().hex()[:12]
        self.on_goal(self.pending)   # 真实 id 只能在这里拿：send 是异步的
        goal_id = self.pending
        if self.cancel_pending:
            handle.cancel_goal_async()
        handle.get_result_async().add_done_callback(lambda done: self._result(done, goal_id))

    def _result(self, future, goal_id):
        self.on_status(STATUS.get(future.result().status, 'ABORTED'), goal_id)
