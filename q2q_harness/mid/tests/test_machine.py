"""状态机离线事件表（10-24 第二轮：真值接管）。

对应教程 09（导航动作与可靠验收）+ 13（任务调度与互锁）。
每个用例都是「给状态 + 给事件 + 期望的新状态/动作」，不启动 ROS，秒级可跑。
"""

import q2q_orchestration.machine as sm

WINDOW = (0.86, 0.90, 0.02, 0.04)      # xmin, xmax, |y|, |yaw|
CORRIDOR = (0.20, 1.20, -0.60, 0.60)


def conds(x=0.0, y=0.0, yaw=0.0, fresh=True, stopped=True, stowed=True):
    return dict(x=x, y=y, yaw=yaw, fresh=fresh, stopped=stopped, stowed=stowed,
                speed=0.0, rate=0.0)


def started(timeout_s=90.0):
    """一个新任务：START 已受理、已发出导航 goal。"""
    machine = sm.Machine(CORRIDOR, WINDOW)
    actions = machine.start(10.0, 'req-1', (0.85, 0.0, 0.0), timeout_s, conds())
    assert actions == [('send_goal', (0.85, 0.0, 0.0))]
    return machine


def test_start_sends_goal_once():
    machine = started()
    assert machine.state == sm.NAV_TO_PICK
    assert machine.start(11.0, 'req-1', (0.85, 0.0, 0.0), 90.0, conds()) == []   # 同一 id 不重发


def test_truth_arrival_cancels_nav_and_takes_over():
    """真值过线就自己叫停：不再等 Nav2 会不会开到 0.921 顶台面。"""
    machine = started()
    actions = machine.tick(12.0, conds(x=0.81))
    assert machine.state == sm.WAIT_STILL
    assert ('cancel_goal',) in actions and ('stop_base',) in actions
    assert machine.expected_cancel is True


def test_own_cancel_result_is_not_late():
    machine = started()
    machine.tick(12.0, conds(x=0.81))
    machine.nav_result('CANCELED', machine.active_goal)
    assert machine.late == 0                  # 自己叫停的终态不算迟到
    assert machine.nav_status == 'CANCELED'


def test_nav_aborted_near_station_is_recoverable():
    """0.921 顶台面那次：abort 时车已经在工位附近，应该交给真值对接，不是直接判死。"""
    machine = started()
    machine.nav_result('ABORTED', machine.active_goal)
    assert machine.state == sm.NAV_TO_PICK     # 还没判死
    machine.tick(13.0, conds(x=0.78))
    assert machine.state == sm.WAIT_STILL


def test_nav_aborted_far_away_fails():
    machine = started()
    machine.nav_result('ABORTED', machine.active_goal)
    machine.tick(13.0, conds(x=0.30))
    assert machine.state == sm.FAILED
    assert machine.reason == 'nav_aborted'


def test_nav_timeout_still_cancels():
    machine = started(timeout_s=90.0)
    actions = machine.tick(101.0, conds(x=0.30))
    assert machine.state == sm.CANCELING
    assert actions == [('cancel_goal',)]


def test_wait_still_docks_then_locks():
    machine = started()
    machine.nav_result('SUCCEEDED', machine.active_goal)
    machine.tick(20.0, conds(x=0.72))                  # 停稳了 → 先落 WAIT_STILL
    assert machine.state == sm.WAIT_STILL
    machine.tick(20.1, conds(x=0.72))                  # 停稳但没进窗口 → 交给真值对接
    assert machine.state == sm.DOCKING
    machine.tick(21.0, conds(x=0.88))                  # 对接推进到窗口
    assert machine.state == sm.PICK_READY
    assert machine.state == sm.PICK_READY


def test_reject_is_terminal_failure():
    machine = started()
    machine.nav_result('REJECTED', machine.active_goal)
    assert machine.state == sm.FAILED


def test_picking_survives_a_slow_world():
    """接触让仿真变慢时 odom 会断几秒——那不是「底盘动了」，不许把抓取误杀。"""
    machine = started()
    machine.picking_stale = 30.0                       # 本节用 30 s 演示判据，生产默认 600 s
    machine.nav_result('SUCCEEDED', machine.active_goal)
    machine.tick(20.0, conds(x=0.72))
    machine.tick(20.1, conds(x=0.72))
    machine.tick(21.0, conds(x=0.88))                  # DOCKING → PICK_READY
    machine.tick(21.1, conds(x=0.88))                  # PICK_READY → PICKING（发 grasp）
    assert machine.state == sm.PICKING
    machine.tick(30.0, conds(x=0.88, fresh=False))     # 世界卡了 9 s，数据不新鲜
    assert machine.state == sm.PICKING                 # 但底盘锁着、没位移 → 不判死
    machine.tick(60.0, conds(x=0.88, fresh=False))     # 超过 picking_stale → 才判死
    assert machine.state == sm.CANCELING
    assert machine.grasp_result(False, 'canceled') == [('unlock',)]
    assert machine.state == sm.FAILED


def test_picking_still_dies_on_real_drift():
    machine = started()
    machine.nav_result('SUCCEEDED', machine.active_goal)
    machine.tick(20.0, conds(x=0.72))
    machine.tick(20.1, conds(x=0.72))
    machine.tick(21.0, conds(x=0.88))
    machine.tick(21.1, conds(x=0.88))
    machine.tick(22.0, conds(x=0.90))                  # 位移 2 cm > 5 mm
    assert machine.state == sm.CANCELING
    assert machine.grasp_result(False, 'canceled') == [('unlock',)]
    assert machine.state == sm.FAILED
    assert machine.reason == 'precondition_lost_while_picking'


def test_transfer_cancel_reaches_the_active_controller():
    machine = started()
    machine.state = sm.NAV_TO_PLACE
    assert machine.cancel() == [('cancel_goal',)]
    assert machine.state == sm.CANCELING
    machine.nav_result('CANCELED')
    assert machine.state == sm.IDLE


def test_arm_cancel_keeps_base_locked_until_terminal():
    machine = started()
    machine.state = sm.STOWING
    actions = machine.cancel()
    assert ('cancel_arm',) in actions and ('unlock',) not in actions
    assert machine.state == sm.CANCELING
    assert machine.arm_canceled() == [('unlock',)]
    assert machine.state == sm.IDLE


def test_grasp_cancel_waits_for_confirmed_stop():
    machine = started()
    machine.state = sm.PICKING
    assert ('unlock',) not in machine.cancel()
    assert machine.grasp_result(False, 'grasp_stop_unconfirmed') == [('stop_base',)]
    assert machine.state == sm.FAILED


def test_place_navigation_has_its_own_timeout():
    machine = started(timeout_s=20.)
    machine.state, machine.started = sm.NAV_TO_PLACE, 100.
    assert machine.tick(121., conds()) == [('cancel_goal',)]
    assert machine.state == sm.CANCELING


def test_stow_backs_out_before_return_navigation():
    machine = started()
    machine.state = sm.STOWING
    assert machine.stow_result(True) == [('unlock',)]
    assert machine.state == sm.UNDOCKING
    machine.undock_started = 100.
    assert machine.tick(101., conds(x=.75)) == []
    assert machine.tick(102., conds(x=.63, stopped=False)) == []
    actions = machine.tick(103., conds(x=.63))
    assert machine.state == sm.NAV_TO_PLACE
    assert actions == [('stop_base',), ('send_goal', machine.place)]


def test_place_locks_base_and_done_releases_it():
    machine = started()
    machine.state, machine.nav_status = sm.NAV_TO_PLACE, 'SUCCEEDED'
    machine.started = 100.
    assert machine.tick(101., conds(x=.1)) == [('lock',), ('release',)]
    assert machine.release_result(True) == [('unlock',)]
    assert machine.state == sm.DONE


def test_arrival_must_wait_for_nav_cancellation_terminal():
    machine = started()
    machine.tick(12., conds(x=.81))
    assert machine.tick(13., conds(x=.88)) == []
    assert machine.state == sm.WAIT_STILL
    machine.nav_result('CANCELED')
    assert machine.tick(14., conds(x=.88)) == [('lock',)]


def test_abort_during_own_arrival_cancel_is_terminal():
    machine = started()
    machine.tick(12., conds(x=.81))
    machine.nav_result('ABORTED')
    assert not machine.expected_cancel
    assert machine.tick(14., conds(x=.88)) == [('lock',)]
