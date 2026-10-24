"""状态机：状态 + 事件 → 新状态 + 动作清单。不依赖 rclpy，可离线跑事件表。

事件表：docs/decisions/2026-10-24-第7步接口定稿-第二片.md §4（第一、二片）、
第三片 §2（DOCKING / PICK_READY）。**改逻辑之前先改表**，离线断言跟着表走。

动作是元组，节点负责执行：('send_goal', (x,y,yaw)) / ('cancel_goal',) /
('stop_base',) / ('lock',) / ('unlock',)。

> 行数说明：这个文件刻意保持成「一张能读下来的事件表」，拆文件反而更难核对；
> 当前略超项目 80 行标准，已在当日日志记为代码债。
"""

IDLE = 'IDLE'
NAV_TO_PICK = 'NAV_TO_PICK'
WAIT_STILL = 'WAIT_STILL'
DOCKING = 'DOCKING'
PICK_READY = 'PICK_READY'
PICKING = 'PICKING'
PICKED = 'PICKED'
STOWING = 'STOWING'
UNDOCKING = 'UNDOCKING'
NAV_TO_PLACE = 'NAV_TO_PLACE'
PLACE_READY = 'PLACE_READY'
DONE = 'DONE'
CANCELING = 'CANCELING'
FAILED = 'FAILED'

from .transport import TransportMixin  # noqa: E402  （取放后半段，避免单文件过长）


class Machine(TransportMixin):
    def __init__(self, corridor=(0.20, 1.20, -0.60, 0.60), dock_window=(0.86, 0.90, 0.02, 0.04)):
        self.corridor, self.dock_window = corridor, dock_window
        self.state, self.generation = IDLE, 0
        self.reason, self.nav_status = '', 'NONE'
        self.late, self.distance = 0, -1.0
        self.active_goal, self.last_request = '', ''
        self.started, self.timeout, self.cancel_sent = None, 0.0, False
        self.dock_started, self.dock_timeout = None, 20.0
        self.place = (0.10, 0.0, 0.0)      # 投放点（map 帧）；由节点参数覆盖
        self.base_ref = None               # 进 PICKING 时的底盘位姿，用来判「有没有被推动」
        # 真值接管（10-24 两次整链实跑加的）：Nav2 对 0.85 这个目标会开过头顶到台面再 abort，
        # 所以不能只信它的终态。x 过线就自己叫停，由真值短距对接收最后几厘米。
        self.arrive_x, self.recover_band = 0.80, 0.10
        self.expected_cancel = False       # 这次取消是我们自己发起的，终态不算迟到
        self.picking_stale = 10.0          # Bounded telemetry-loss window; not a motion budget.
        self.picking_fresh_at = None

    def _enter(self, state, reason):
        self.state, self.reason = state, reason
        self.generation += 1

    def in_window(self, conds):
        """是否已经站在抓取走廊里（第三片的核心判据，用真值算）。"""
        xmin, xmax, dy, dyaw = self.dock_window
        return (xmin <= conds['x'] <= xmax and abs(conds['y']) <= dy
                and abs(conds['yaw']) <= dyaw)

    def start(self, now, request_id, target, timeout_s, conds):
        if self.state not in (IDLE, FAILED, DONE):
            self.reason = 'busy:%s' % self.state      # 上一轮没结束，不许开新任务
            return []
        if request_id == self.last_request:
            return []                                  # 同一 request_id 只处理一次
        self.last_request = request_id
        if not conds['stowed']:
            self.reason = 'arm_not_stowed'             # 练习 13-A：臂没收拢不许开动
            return []
        x, y, _ = target
        xmin, xmax, ymin, ymax = self.corridor
        if not (xmin <= x <= xmax and ymin <= y <= ymax):
            self.reason = 'target_out_of_corridor'
            return []
        self.started, self.timeout, self.cancel_sent = now, timeout_s, False
        self.nav_status, self.distance = 'ACTIVE', -1.0
        self.expected_cancel = False
        self._enter(NAV_TO_PICK, 'nav_started')
        return [('send_goal', target)]

    def cancel(self):
        if self.state in (NAV_TO_PICK,):
            return self._to_canceling('cancel_requested')
        if self.state == DOCKING:
            self._enter(IDLE, 'canceled_in_dock')
            return [('stop_base',)]
        if self.state == WAIT_STILL:
            self._enter(IDLE, 'canceled_in_wait')
            self.nav_status = 'NONE'
        elif self.state in (PICK_READY, PICKED):
            self._enter(IDLE, 'canceled_in_pick')
            return [('unlock',)]
        elif self.state == PICKING:
            self._enter(CANCELING, 'cancel_grasp')
            return [('cancel_grasp',), ('stop_base',)]
        elif self.state == UNDOCKING:
            self._enter(IDLE, 'canceled_in_undock')
            return [('stop_base',)]
        elif self.state == NAV_TO_PLACE:
            return self._to_canceling('cancel_transfer_navigation')
        elif self.state in (STOWING, PLACE_READY):
            self._enter(CANCELING, 'cancel_arm')
            return [('cancel_arm',), ('stop_base',)]
        return []

    def nav_result(self, status, goal_id=''):
        """status: SUCCEEDED / CANCELED / ABORTED / REJECTED。"""
        if goal_id and self.active_goal and goal_id != self.active_goal:
            self.late += 1                             # 不匹配的结果只留痕，不推进
            return []
        if self.expected_cancel and status in ('CANCELED', 'SUCCEEDED', 'ABORTED'):
            # 我们自己因为「已到走廊」叫停的，终态是我们叫出来的，不记迟到
            self.expected_cancel, self.nav_status = False, status
            return []
        if self.state == CANCELING:
            self.nav_status = status
            if status == 'SUCCEEDED':
                self.late += 1                         # 取消前已经成功：不推进但要留痕
                self._enter(IDLE, 'late_success_after_cancel')
            elif status == 'CANCELED':
                self._enter(IDLE, 'canceled')
            else:
                self._enter(FAILED, 'aborted_during_cancel')
            return []
        if self.state == NAV_TO_PLACE:
            # 去投放点那一段也是导航，结果必须记账（否则 tick 永远等不到 SUCCEEDED）
            self.nav_status = status
            if status != 'SUCCEEDED':
                self._enter(FAILED, 'nav_to_place_' + status.lower())
                return [('unlock',)]
            return []
        if self.state != NAV_TO_PICK:
            self.late += 1
            return []
        self.nav_status = status
        if status == 'REJECTED':
            self._enter(FAILED, 'goal_rejected')
        elif status == 'SUCCEEDED':
            self.reason = 'waiting_stop'               # 停稳由 tick 用物理观测判
        elif status == 'ABORTED':
            # 不立刻判死：0.921 那次就是「顶到台面再 abort」，真值其实已经到工位了
            self.reason = 'nav_aborted_wait_truth'
        else:
            self._enter(FAILED, 'nav_' + status.lower())
        return []

    def tick(self, now, conds):
        stopped = conds['fresh'] and conds['stopped']
        if self.state == NAV_TO_PICK:
            if conds['fresh'] and conds['x'] >= self.arrive_x:
                # 真值已进抓取走廊：立刻叫停 Nav2，剩下几厘米交给真值短距对接（DOCKING）
                self._enter(WAIT_STILL, 'arrived_in_corridor')
                if self.nav_status in ('ABORTED', 'SUCCEEDED'):
                    return [('stop_base',)]
                self.expected_cancel = True
                return [('cancel_goal',), ('stop_base',)]
            if self.nav_status == 'ABORTED':
                if conds['fresh'] and conds['x'] >= self.arrive_x - self.recover_band:
                    self._enter(WAIT_STILL, 'nav_aborted_near_station')
                    return [('stop_base',)]
                self._enter(FAILED, 'nav_aborted')
                return [('stop_base',)]
            if self.nav_status == 'SUCCEEDED':
                if stopped:
                    self._enter(WAIT_STILL, 'stopped_at_station')
                else:
                    self.reason = 'waiting_stop'
            elif self.timeout > 0 and self.started is not None and now - self.started > self.timeout:
                self.nav_status = 'TIMEOUT'
                return self._to_canceling('nav_timeout')
        elif self.state == WAIT_STILL:
            if self.expected_cancel:
                self.reason = 'waiting_nav_terminal'
                return []  # Do not dock while the previous navigator can still command velocity.
            if self.in_window(conds) and stopped:
                self._enter(PICK_READY, 'in_corridor')
                return [('lock',)]
            if not self.in_window(conds) and stopped:
                self.dock_started = now
                self._enter(DOCKING, 'docking')
            else:
                self.reason = 'waiting_stop'
        elif self.state == DOCKING:
            if conds['x'] > self.dock_window[1] + 0.02:
                self._enter(FAILED, 'dock_overshoot')
                return [('stop_base',)]
            if self.in_window(conds) and stopped:
                self._enter(PICK_READY, 'docked')
                return [('stop_base',), ('lock',)]
            if self.dock_started is not None and now - self.dock_started > self.dock_timeout:
                self._enter(FAILED, 'dock_timeout')
                return [('stop_base',)]
            self.reason = 'docking'
        elif self.state == PICK_READY:
            # 四条前置（新鲜/停稳/已锁/臂收拢）任一条丢了就必须退出去并解锁
            if not (conds['fresh'] and conds['stopped'] and conds['stowed']):
                self._enter(FAILED, 'precondition_lost')
                return [('unlock',)]
            # 四条都成立才动臂，而且只发一次 grasp（第四片）
            self.base_ref = (conds['x'], conds['y'], conds['yaw'])
            self.picking_fresh_at = now
            self._enter(PICKING, 'picking')
            return [('grasp',)]
        elif self.state == PICKING:
            # 臂正在动，所以这里**不再要求 arm_stowed**（那本来就会变 false）。
            # **也不能要求「连续停稳」**：真值 odom 的角速度噪声会让 0.5 s 窗口反复清零，
            # 实测进 PICKING 后 0.2 s 就把自己的抓取取消了（10-24 1432 那次）。
            # 正确判据是「底盘相对进 PICKING 时**没有位移**」——直接量，不看速度。
            # **新鲜度也不能用 1.5 s 的通用窗口**：接触让仿真变慢时 odom 墙钟间隔会长到几秒，
            # 那不是"底盘动了"，是"世界卡了一下"（10-24 1757 实测把抓取误杀）。
            # 这里改成专门的容忍值：底盘已上锁，只有真的几秒没数据才判死。
            self.reason = 'picking'
            if conds['fresh']:
                self.picking_fresh_at = now
            stale_too_long = (self.picking_fresh_at is not None
                              and now - self.picking_fresh_at > self.picking_stale)
            drifted = (self.base_ref is not None and
                       ((conds['x'] - self.base_ref[0]) ** 2 +
                        (conds['y'] - self.base_ref[1]) ** 2) ** 0.5 > 0.005)
            if stale_too_long or drifted:
                self._enter(CANCELING, 'cancel_grasp_precondition')
                return [('cancel_grasp',), ('stop_base',)]
        else:
            return self.tick_transport(now, conds) or []   # 必须返回列表：节点直接迭代它
        return []

    def grasp_result(self, success, reason=''):
        """流水线的结论：只认它自己给的 success/reason，不在这里编。"""
        if self.state == CANCELING and self.reason.startswith('cancel_grasp'):
            if reason == 'canceled':
                failed = self.reason == 'cancel_grasp_precondition'
                self._enter(FAILED if failed else IDLE,
                            'precondition_lost_while_picking' if failed else 'grasp_canceled')
                return [('unlock',)]
            self._enter(FAILED, reason or 'grasp_stop_unconfirmed')
            return [('stop_base',)]
        if self.state != PICKING:
            self.late += 1
            return []
        if success:
            self._enter(PICKED, 'picked_hold')
            return []
        self._enter(FAILED, reason or 'grasp_failed')
        return [('unlock',)]

    def arm_canceled(self):
        if self.state == CANCELING and self.reason == 'cancel_arm':
            self._enter(IDLE, 'arm_canceled')
            return [('unlock',)]
        return []

    def fail(self, reason):
        """外部原因判失败（例如导航服务迟迟不就绪）；状态机自己不做这个判断。"""
        self._enter(FAILED, reason)
        return [('stop_base',)]

    def _to_canceling(self, reason):
        self._enter(CANCELING, reason)
        if self.cancel_sent:
            return []
        self.cancel_sent = True
        return [('cancel_goal',)]
