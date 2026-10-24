"""导航就绪闸门：START 先暂存，等 action 服务与真值都就绪再放行。

为什么要它（10-24 两次整链实跑）：
- 冷启动时 Nav2 还没起 action 服务，goal 发出去没人接，白白烧掉 90 s 导航超时，
  状态机最后记的是 `nav_timeout` —— 看起来像「导航能力不行」，其实只是**发早了**；
- 反过来，只要 goal 真的被接受，就说明环境已就绪，超时才有意义。

所以「发 goal」这一步必须先过闸门：服务不在就等着，等不到才算失败（有自己的超时，
不占用导航预算）。纯逻辑，不依赖 rclpy，可离线断言。
"""


class ReadyGate:
    def __init__(self, timeout_s):
        if timeout_s <= 0:
            raise ValueError('就绪超时必须为正')
        self.timeout_s, self.pending, self.queued_at = timeout_s, None, None

    def offer(self, message, now, ready):
        """来了一条 START：就绪就原样放行（返回它），否则暂存并返回 None。"""
        if ready:
            return message
        self.pending, self.queued_at = message, now
        return None

    def poll(self, now, ready):
        """定时器每拍调用：返回 (要放行的命令 或 None, 是否已等超时)。"""
        if self.pending is None:
            return None, False
        if ready:
            message, self.pending = self.pending, None
            return message, False
        if now - self.queued_at > self.timeout_s:
            self.pending = None
            return None, True
        return None, False
