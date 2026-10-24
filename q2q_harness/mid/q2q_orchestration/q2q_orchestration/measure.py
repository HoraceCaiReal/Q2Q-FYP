"""把 odom/joint_states 变成四个条件。纯计算，不依赖 rclpy，便于离线测。

时钟约定（第一片实测教训）：`header.stamp` 用节点钟（可能是仿真钟），
但「新鲜度 / 停稳窗口」用**墙钟**——仿真暂停时 /clock 也停，用仿真钟永远判不出 stale。
"""

import math

ARM_JOINTS = ['arm_0_joint_%d' % i for i in range(1, 7)]


def yaw_from(q):
    """四元数 → 偏航。w=0 是合法的 180°，不能当缺省值替换（10-24 修过的坑）。"""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class WorldMeasure:
    def __init__(self, fresh_window, stop_window, v_eps, w_eps, stowed_rad, joints_window=0.5):
        self.fresh_window, self.stop_window = fresh_window, stop_window
        self.v_eps, self.w_eps, self.stowed_rad = v_eps, w_eps, stowed_rad
        self.joints_window = joints_window
        self.odom = self.odom_at = self.joints_at = None
        self.stowed, self.stopped_since = False, None

    def on_odom(self, msg, now):
        self.odom, self.odom_at = msg, now
        speed = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)
        still = speed <= self.v_eps and abs(msg.twist.twist.angular.z) <= self.w_eps
        self.stopped_since = (self.stopped_since or now) if still else None

    def on_joints(self, msg, now):
        self.joints_at = now
        values = dict(zip(msg.name, msg.position))
        self.stowed = all(abs(values.get(j, math.inf)) <= self.stowed_rad for j in ARM_JOINTS)

    def conditions(self, now):
        """四个条件 + 观测值。UNKNOWN 一律按「不满足」处理（教程 13 章）。"""
        fresh = self.odom_at is not None and now - self.odom_at <= self.fresh_window
        stopped = self.stopped_since is not None and now - self.stopped_since >= self.stop_window
        joints_fresh = self.joints_at is not None and now - self.joints_at <= self.joints_window
        out = dict(fresh=fresh, stopped=stopped, stowed=bool(joints_fresh and self.stowed),
                   x=0.0, y=0.0, yaw=0.0, speed=0.0, rate=0.0)
        if self.odom is not None:
            pose, twist = self.odom.pose.pose, self.odom.twist.twist
            out.update(x=pose.position.x, y=pose.position.y, yaw=yaw_from(pose.orientation),
                       speed=math.hypot(twist.linear.x, twist.linear.y), rate=twist.angular.z)
        return out

def bounded_param(node, name, low, high):
    """读参数并校验：越界就启动失败，不允许默认值悄悄生效。"""
    value = node.get_parameter(name).value
    if not low < value <= high:
        raise SystemExit('%s=%s 越界（要求 %s~%s）；下一步该查：启动参数' % (name, value, low, high))
    return value
