"""取放后半段（收臂 → 去投放点 → 松爪 → DONE）的转移，做成 Mixin 与主状态机分开。

为什么拆：主状态机要保持「一张能读下来的事件表」，而这段是**运输/投放**的独立关注点，
两个都塞进一个文件会到 200 行且读不动。约定：Mixin 只提供被 `Machine.tick` 调用的分支，
自身不持有额外状态（状态字段仍归 Machine）。
"""

from .machine import DONE, FAILED, NAV_TO_PLACE, PICKED, PLACE_READY, STOWING, UNDOCKING


class TransportMixin:
    def tick_transport(self, now, conditions):
        """处理 PICKED 之后的状态；本 Mixin 不负责的状态返回 None。"""
        stopped = conditions['fresh'] and conditions['stopped']
        if self.state == PICKED:
            # 拿到物体后先收臂（底盘仍锁着），收完才允许开车
            self._enter(STOWING, 'stowing')
            return [('stow',)]
        if self.state == STOWING:
            self.reason = 'stowing'
        elif self.state == UNDOCKING:
            if now-self.undock_started > 90. or abs(conditions['y']) > .04 or abs(conditions['yaw']) > .08:
                self._enter(FAILED, 'undock_timeout_or_corridor_lost')
                return [('stop_base',)]
            if conditions['fresh'] and conditions['stowed'] and conditions['x'] <= .64 and stopped:
                self.nav_status, self.active_goal, self.expected_cancel = 'ACTIVE', '', False
                self._enter(NAV_TO_PLACE, 'nav_to_place')
                return [('stop_base',), ('send_goal', self.place)]
        elif self.state == NAV_TO_PLACE:
            if self.started is not None and now-self.started > self.timeout:
                return self._to_canceling('nav_to_place_timeout')
            if self.nav_status == 'SUCCEEDED':
                if stopped:
                    self._enter(PLACE_READY, 'at_place')
                    return [('lock',), ('release',)]
                self.reason = 'waiting_stop_at_place'
            else:
                self.reason = 'nav_to_place'
        elif self.state == PLACE_READY:
            self.reason = 'placing'
        elif self.state == DONE:
            self.reason = 'done'
        return None

    def stow_result(self, ok, reason=''):
        """收臂完成：解锁底盘，然后去投放点。"""
        if self.state != STOWING:
            self.late += 1
            return []
        if not ok:
            self._enter(FAILED, reason or 'stow_failed')
            return [('unlock',)]
        self.nav_status = 'ACTIVE'
        self.active_goal = ''
        self.started = None  # Node assigns a fresh wall deadline for this navigation leg.
        self.expected_cancel = False          # 新的一段导航，先把「预期取消」清掉
        self.undock_started = 0.  # Node stamps the wall clock in on_stow.
        self._enter(UNDOCKING, 'undocking')
        return [('unlock',)]

    def release_result(self, ok, reason=''):
        """松爪完成 → 一轮任务结束。"""
        if self.state != PLACE_READY:
            self.late += 1
            return []
        if ok:
            self._enter(DONE, 'done')
            return [('unlock',)]
        self._enter(FAILED, reason or 'release_failed')
        return [('unlock',)]
