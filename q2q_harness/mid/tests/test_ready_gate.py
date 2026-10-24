"""就绪闸门的离线事件表：冷启动时 START 不该白烧导航超时。"""

import pytest

from q2q_orchestration.ready_gate import ReadyGate


def test_ready_start_goes_straight_through():
    gate = ReadyGate(45.0)
    assert gate.offer('START', now=0.0, ready=True) == 'START'
    assert gate.pending is None


def test_not_ready_start_is_queued_then_released():
    gate = ReadyGate(45.0)
    assert gate.offer('START', now=0.0, ready=False) is None
    assert gate.poll(now=1.0, ready=False) == (None, False)
    assert gate.poll(now=2.0, ready=True) == ('START', False)   # 服务起来就放行
    assert gate.pending is None


def test_queue_expires_with_its_own_budget():
    gate = ReadyGate(5.0)
    gate.offer('START', now=0.0, ready=False)
    assert gate.poll(now=5.0, ready=False) == (None, False)     # 正好 5 s 还不算超
    assert gate.poll(now=5.1, ready=False) == (None, True)      # 超了自己的预算
    assert gate.pending is None


def test_two_starts_keep_only_the_last():
    gate = ReadyGate(45.0)
    gate.offer('START-1', now=0.0, ready=False)
    gate.offer('START-2', now=1.0, ready=False)
    assert gate.poll(now=2.0, ready=True) == ('START-2', False)


def test_timeout_must_be_positive():
    with pytest.raises(ValueError):
        ReadyGate(0)
