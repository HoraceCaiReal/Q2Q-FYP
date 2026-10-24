#!/usr/bin/env python3
"""Reconcile a lost map-server startup reply by actual lifecycle state, once.

Nav2's normal lifecycle manager remains enabled. This startup guard only activates
a map server that stayed configured/inactive for five seconds; it never resends a
transition based solely on a missing reply. Not a runtime safety watchdog.
"""
import time
import rclpy
from lifecycle_msgs.srv import GetState, ChangeState
from lifecycle_msgs.msg import Transition
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
import action_sync


def main():
    rclpy.init()
    node = rclpy.create_node('map_startup_guard')
    seen = {}
    node.create_subscription(OccupancyGrid, '/a200_0000/map',
        lambda m: seen.update(valid=m.info.width*m.info.height == len(m.data) and len(m.data)>0),
        QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                   durability=DurabilityPolicy.TRANSIENT_LOCAL))
    get = node.create_client(GetState, '/a200_0000/map_server/get_state')
    put = node.create_client(ChangeState, '/a200_0000/map_server/change_state')
    inactive_since, attempted = None, False
    try:
        if not get.wait_for_service(timeout_sec=60.) or not put.wait_for_service(timeout_sec=5.):
            raise RuntimeError('map_server lifecycle services unavailable')
        deadline = time.monotonic()+60.
        while time.monotonic() < deadline:
            try:
                state = action_sync.wait(node, get.call_async(GetState.Request()), 2.).current_state.id
            except TimeoutError:
                continue
            if state == 3 and seen.get('valid'):
                node.get_logger().info('Map server ACTIVE and OccupancyGrid received')
                return 0
            if state == 2:
                inactive_since = inactive_since or time.monotonic()
                if not attempted and time.monotonic()-inactive_since >= 5.:
                    attempted = True
                    req = ChangeState.Request()
                    req.transition.id = Transition.TRANSITION_ACTIVATE
                    node.get_logger().warn('Map stayed INACTIVE; reconciling activation once')
                    try:
                        action_sync.wait(node, put.call_async(req), 4.)
                    except TimeoutError:
                        pass  # Next get_state + map receipt determine the actual outcome.
            else:
                inactive_since = None
            rclpy.spin_once(node, timeout_sec=.2)
        raise TimeoutError('Map startup not verified; navigation must remain gated')
    except Exception as error:
        node.get_logger().error(str(error))
        return 2
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
