"""真值短距对接：用 ground_truth 闭环把车推进抓取走廊，并保证「退出即零速」。

为什么单独一个文件：对接控制要 20 Hz 跑，状态发布只要 10 Hz；两条定时器混在一个回调里，
最容易写出「已经离开 DOCKING 还在发速度」这类 bug。这里把「发速度」这一件事单独关起来，
并且**离开状态时必定补一帧零速**（不是靠超时兜底）。

为什么用真值而不是 Nav2：第二片实测导航目标 0.85 停在 0.717（容差 13 cm），
抓取走廊要 0.86–0.90。最后这一段必须按物理真值闭环，不能拿导航容差去兜抓取精度。
"""

from geometry_msgs.msg import Twist


class Docker:
    def __init__(self, node, topic, window, speed):
        self.window, self.speed = window, speed
        self.publisher = node.create_publisher(Twist, topic, 1)
        self.active = False

    def update(self, docking, conditions):
        """docking=状态机是否在 DOCKING；conditions 来自 WorldMeasure（真值）。"""
        if not docking:
            if self.active:
                self._publish(0.0)
                self.active = False
            return
        self.active = True
        if not conditions['fresh'] or not conditions['stowed']:
            self._publish(0.)
            return
        target = 0.5 * (self.window[0] + self.window[1])
        error = target - conditions['x']
        if abs(error) < 0.005:
            self._publish(0.0)                         # 进入死区就停，别在窗口里来回蹭
            return
        self._publish(max(-self.speed, min(self.speed, 1.5 * error)))

    def _publish(self, speed):
        message = Twist()
        message.linear.x = float(speed)
        self.publisher.publish(message)
