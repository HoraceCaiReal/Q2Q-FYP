"""给手臂目标做规划：只负责「要一条轨迹」，不负责执行。

输入：6 个手臂关节的目标位置 + 速度系数。
输出：(trajectory, 原因)。规划器、碰撞、限位都交给 MoveIt，这里不自己算 IK。
"""

JOINTS = ['arm_0_joint_%d' % i for i in range(1, 7)]
PLAN_SERVICE = '/a200_0000/plan_kinematic_path'


async def bounded(node, pending, seconds):
    """Wall-clock deadline; caller uses a single executor thread for timer safety."""
    import time
    from rclpy.clock import Clock, ClockType
    from rclpy.task import Future
    result = Future()
    deadline = time.monotonic()+seconds
    def poll():
        if result.done():
            return
        if pending.done():
            if pending.exception():
                result.set_exception(pending.exception())
            else:
                result.set_result(pending.result())
        elif time.monotonic() >= deadline:
            result.set_exception(TimeoutError('Arm response exceeded wall deadline'))
    timer = node.create_timer(.05, poll, clock=Clock(clock_type=ClockType.STEADY_TIME))
    try:
        poll()
        return await result
    finally:
        node.destroy_timer(timer)


def build_request(target, vel_scale):
    from moveit_msgs.msg import Constraints, JointConstraint, MotionPlanRequest

    request = MotionPlanRequest()
    request.group_name = 'arm'
    request.num_planning_attempts = 5
    request.allowed_planning_time = 5.0
    request.start_state.is_diff = True
    request.max_velocity_scaling_factor = vel_scale
    request.max_acceleration_scaling_factor = vel_scale
    request.goal_constraints = [Constraints(joint_constraints=[
        JointConstraint(joint_name=j, position=p, tolerance_above=0.02,
                        tolerance_below=0.02, weight=1.0)
        for j, p in zip(JOINTS, target)])]
    return request


async def plan(client, target, vel_scale, node):
    """异步调用规划服务；返回 (trajectory | None, 原因)。"""
    from moveit_msgs.srv import GetMotionPlan

    if not client.service_is_ready():
        return None, 'plan_kinematic_path 不在：move_group 起了吗'
    request = GetMotionPlan.Request()
    request.motion_plan_request = build_request(target, vel_scale)
    response = await bounded(node, client.call_async(request), 15.)
    if response is None:
        return None, '规划服务超时'
    code = response.motion_plan_response.error_code.val
    if code != 1:
        return None, '规划失败 error_code=%d' % code
    trajectory = response.motion_plan_response.trajectory.joint_trajectory
    if not trajectory.points:
        return None, '规划返回空轨迹'
    return trajectory, 'ok'
