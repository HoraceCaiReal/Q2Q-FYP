"""把「状态机 + 测量」装配成 TaskState。只做字段填充，不发布、不做决策。

放在单独文件的原因是：装配逻辑一膨胀就会把节点文件顶过行数上限，
而它是纯函数（给定输入→给定消息），可以单独读、单独测。
"""

from q2q_interfaces.msg import TaskState

from . import machine as sm


def build(machine, conds, episode, task_id, stamp):
    out = TaskState()
    out.header.stamp, out.header.frame_id = stamp, 'map'
    out.episode_id, out.task_id = episode, task_id
    out.generation, out.state = machine.generation, machine.state
    out.base_fresh, out.base_stopped, out.arm_stowed = conds['fresh'], conds['stopped'], conds['stowed']
    # 三个占位字段：这一片确实没有对应的测量或控制，写 false 而不是猜
    out.base_command_locked = out.arm_executing = out.holding_object = False
    out.base_x, out.base_y, out.base_yaw = conds['x'], conds['y'], conds['yaw']
    out.base_speed, out.base_yaw_rate = conds['speed'], conds['rate']
    out.nav_goal_id, out.nav_status = machine.active_goal, machine.nav_status
    out.distance_remaining = machine.distance
    out.last_request_id, out.late_events = machine.last_request, machine.late
    out.blocked_by = machine.reason if machine.state != sm.IDLE else ''
    return out
