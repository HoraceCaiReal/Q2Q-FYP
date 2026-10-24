"""Bound async ROS futures by steady wall time, including paused simulation."""
import time
from rclpy.clock import Clock, ClockType
from rclpy.task import Future


async def bounded(node, pending, seconds):
    result = Future()
    deadline = time.monotonic() + seconds

    def check():
        if result.done():
            return
        if pending.done():
            exception = pending.exception()
            if exception:
                result.set_exception(exception)
            else:
                result.set_result(pending.result())
        elif time.monotonic() >= deadline:
            result.set_exception(TimeoutError('ROS response exceeded wall-time budget'))

    timer = node.create_timer(.05, check, clock=Clock(clock_type=ClockType.STEADY_TIME))
    try:
        check()
        return await result
    finally:
        node.destroy_timer(timer)
