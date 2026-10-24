"""One action at a time; result timeouts cancel, never silently leave motion alive."""
import os

import rclpy

# 墙钟预算的仿真速度补偿：这台机器 RTF 会在 0.1–0.3 之间飘，同一个 3 s 的等待
# 在 RTF=0.15 时只等于 0.45 s 仿真时间，服务/动作根本来不及应答。
# Q2Q_WALL_SCALE=6 表示所有墙钟预算按「最慢时约慢 6 倍」放大；默认 1.0 不改变老行为。
SCALE = float(os.environ.get('Q2Q_WALL_SCALE', '1.0'))


def wait(node, future, seconds):
    seconds = seconds * SCALE
    rclpy.spin_until_future_complete(node, future, timeout_sec=seconds)
    if not future.done():
        raise TimeoutError('ROS response exceeded %.1f wall seconds' % seconds)
    if future.exception():
        raise future.exception()
    return future.result()


def call(node, client, goal, timeout):
    node._motion_stop_confirmed = False
    pending = client.send_goal_async(goal)
    try:
        handle = wait(node, pending, 8.)
    except TimeoutError:
        # A late acceptance must not become an untracked execution.
        def late(future):
            if not future.exception() and future.result() and future.result().accepted:
                future.result().cancel_goal_async()
        pending.add_done_callback(late)
        # Give a late reply/cancellation a bounded chance; never resend the goal.
        rclpy.spin_until_future_complete(node, pending, timeout_sec=3.)
        raise
    if handle is None or not handle.accepted:
        node._motion_stop_confirmed = True
        raise RuntimeError('Action rejected')
    result = handle.get_result_async()
    try:
        terminal = wait(node, result, timeout)
        node._motion_stop_confirmed = terminal.status in (4, 5, 6)
        return terminal
    except BaseException:
        cancellation = handle.cancel_goal_async()
        reply = wait(node, cancellation, 3.)
        if not reply or not reply.goals_canceling:
            raise RuntimeError('Timeout; cancellation not confirmed, execution UNKNOWN')
        terminal = wait(node, result, 5.)
        if terminal.status not in (4, 5, 6):
            raise RuntimeError('Cancellation terminal status UNKNOWN')
        node._motion_stop_confirmed = True
        raise
