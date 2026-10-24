#!/usr/bin/env python3
"""grasp_io：抓取回路用到的 I/O —— 发一次 action、读一次真值、量一次爪。

只做「发/等/报」，不含任何流程决策；流程在 grasp_run.py。
所有位姿来自 Gazebo 的 `/world/q2q_warehouse/pose/info`，指尖宽度是**量的**、不是 FK 推的。

坐标系（10-24 修正）：顶层模型是 world，机器人 link 是 model 局部坐标。
Jackal 根已设为地面投影 base_footprint；落地 model z≈0，base_link 在其上 63.5 mm。
物体转入 base 时必须使用 model 的完整平移与四元数，不能无条件忽略 z/roll/pitch。
规划和执行分开：MoveGroup 仅 plan_only，实际执行走有超时取消的 JTC。
"""

import time

import numpy as np
from std_msgs.msg import Bool

import grasp_pose
import object_pose
import pose_move
import action_sync
import scene_geometry
import rclpy
from control_msgs.action import GripperCommand, FollowJointTrajectory
from moveit_msgs.action import MoveGroup
from rclpy.action import ActionClient

GRIPPER = '/a200_0000/arm_0_gripper_controller/gripper_cmd'
MOVE = '/a200_0000/move_action'
ROBOT = 'a200_0000/robot'
LEFT = 'arm_0_gripper_left_finger_dist_link'
RIGHT = 'arm_0_gripper_right_finger_dist_link'
EE = 'arm_0_end_effector_link'


def clients(node):
    """(move, grip)：两个 action client，缺任何一个都返回 None。"""
    move = ActionClient(node, MoveGroup, MOVE)
    grip = ActionClient(node, GripperCommand, GRIPPER)
    ready = move.wait_for_server(timeout_sec=15.0) and grip.wait_for_server(timeout_sec=15.0)
    return (move, grip) if ready else None


def gripper_client(node):
    """**只**等夹爪。夹爪自检不该依赖 move_group（10-23 实测：没起 move_group 时
    `clients()` 返回 None，而报错只提夹爪，把排查带偏）。"""
    grip = ActionClient(node, GripperCommand, GRIPPER)
    # Cold generation + controller reconciliation can exceed 25 s. Wait for the
    # actual endpoint within a finite budget, rather than a fixed startup sleep.
    return grip if grip.wait_for_server(timeout_sec=90.0) else None


def call(client, goal, node, timeout):
    """Bounded call; timeout raises and cancels rather than authorizing a retry."""
    return action_sync.call(node, client, goal, timeout)


def suction(node, attach):
    """吸附近似：发一次 Bool 到 `/q2q/low/suction`（true=吸住 / false=松开）。

    等待relay确认支撑状态；物体是否跟手仍须独立真值验收。
    这是运动学支撑，不是物理吸附。
    """
    if not hasattr(node, '_suction_publisher'):
        node._suction_publisher = node.create_publisher(Bool, '/q2q/low/suction', 1)
    confirmed = [None]
    subscription = node.create_subscription(Bool, '/q2q/low/suction_active',
                                            lambda m: confirmed.__setitem__(0, m.data), 1)
    deadline = time.monotonic()+10.
    while node._suction_publisher.get_subscription_count() < 1:
        if time.monotonic() > deadline:
            node.destroy_subscription(subscription)
            raise TimeoutError('Carry relay command endpoint unavailable')
        rclpy.spin_once(node, timeout_sec=.1)
    message = Bool()
    message.data = bool(attach)
    node._suction_publisher.publish(message)
    try:
        while confirmed[0] != bool(attach):
            if time.monotonic() > deadline:
                raise TimeoutError('Carry relay did not acknowledge requested state')
            rclpy.spin_once(node, timeout_sec=.1)
    finally:
        node.destroy_subscription(subscription)
    return {'attach': bool(attach), 'relay_confirmed': True}


def gripper(node, client, position, effort, timeout=55.0):
    """Bounded GripperCommand facade; max_effort is NOT enforced in position simulation."""
    goal = GripperCommand.Goal()
    goal.command.position = float(position)
    goal.command.max_effort = float(effort)
    wrapped = call(client, goal, node, timeout)
    if wrapped is None:
        return {'reached': False, 'timeout_s': timeout}
    return {'reached': bool(wrapped.result.reached_goal),
            'position': round(wrapped.result.position, 4),
            'stalled': bool(wrapped.result.stalled), 'status': wrapped.status,
            'effort_limit_enforced': False}


def move_ee(node, client, point, quaternion, radius=0.015):
    """走到一个末端位置。`radius` 是位置约束球半径——**小步移动必须调小**（≤步长的一半），
    否则起点本身就落在目标域里，规划器会回一个「成功但 0 点」的空计划，看着成功其实没动。"""
    wrapped = call(client, pose_move.build_goal(*point, quaternion, True, radius), node, 15.0)
    summary = {'target': [round(float(v), 4) for v in point], 'radius': radius}
    if wrapped is None:
        return dict(summary, error_code=None, points=0)
    trajectory = wrapped.result.planned_trajectory.joint_trajectory
    summary.update(error_code=wrapped.result.error_code.val, points=len(trajectory.points))
    if wrapped.status != 4 or summary['error_code'] != 1 or not trajectory.points:
        raise RuntimeError('Pose planning failed: '+str(summary))
    execute_trajectory(node, trajectory)
    return summary


def execute_trajectory(node, trajectory):
    """Send exactly once to JTC. Timeouts cancel and abort this run."""
    if not trajectory.points:
        raise RuntimeError('Empty trajectory')
    final = trajectory.points[-1].time_from_start
    seconds = final.sec + final.nanosec/1e9
    if not 0 < seconds < 90:
        raise RuntimeError('Invalid/unsafe trajectory duration: '+str(seconds))
    client = ActionClient(node, FollowJointTrajectory,
        '/a200_0000/arm_0_joint_trajectory_controller/follow_joint_trajectory')
    try:
        if not client.wait_for_server(timeout_sec=5.):
            raise RuntimeError('Arm JTC unavailable')
        # Mesh contact can lower Fortress RTF below .1 in this VM. Simulation
        # duration is not wall duration: permit a measured slow-run budget, bounded.
        # 2026-10-24：5 ms 步长 + 接触的世界里 RTF 更低，原来 60 s 底数会让一次 25 mm 抬升超时
        # （实测 terminal status=5，动作被我们自己取消）。底数提到 120 s、上限 300 s。
        wrapped = call(client, FollowJointTrajectory.Goal(trajectory=trajectory), node,
                       min(300., max(120., seconds*40+30)))
        if wrapped.status != 4 or wrapped.result.error_code != 0:
            raise RuntimeError('Arm execution failed: '+str(wrapped.result))
    finally:
        client.destroy()


def cartesian(node, point, quaternion):
    """Collision-checked straight-line insertion/lift; partial paths are rejected."""
    from geometry_msgs.msg import Pose
    from moveit_msgs.srv import GetCartesianPath
    client = node.create_client(GetCartesianPath, '/a200_0000/compute_cartesian_path')
    try:
        if not client.wait_for_service(timeout_sec=5.):
            raise RuntimeError('Cartesian service unavailable')
        req = GetCartesianPath.Request()
        req.header.frame_id = 'base_footprint'
        req.start_state.is_diff = True
        req.group_name, req.link_name = 'arm', EE
        req.max_step, req.jump_threshold, req.avoid_collisions = .005, 0., True
        req.revolute_jump_threshold = .20
        req.max_velocity_scaling_factor = .10
        req.max_acceleration_scaling_factor = .10
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = map(float, point)
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = quaternion
        req.waypoints = [pose]
        res = action_sync.wait(node, client.call_async(req), 12.)
        if res.error_code.val != 1 or res.fraction < .999:
            raise RuntimeError('Cartesian path incomplete: fraction=%s code=%s' %
                               (res.fraction, res.error_code.val))
        execute_trajectory(node, res.solution.joint_trajectory)
        return dict(error_code=1, fraction=res.fraction,
                    points=len(res.solution.joint_trajectory.points),
                    target_ee=list(map(float, point)))
    finally:
        node.destroy_client(client)


def allow_finger_contact(node, name):
    """Preserve the object/table; permit target contact only at gripper fingers."""
    from moveit_msgs.msg import PlanningScene, PlanningSceneComponents, AllowedCollisionEntry
    from moveit_msgs.srv import GetPlanningScene, ApplyPlanningScene
    get = node.create_client(GetPlanningScene, '/a200_0000/get_planning_scene')
    put = node.create_client(ApplyPlanningScene, '/a200_0000/apply_planning_scene')
    try:
        if not get.wait_for_service(timeout_sec=5.) or not put.wait_for_service(timeout_sec=5.):
            raise RuntimeError('Planning scene services unavailable')
        req = GetPlanningScene.Request()
        req.components.components = PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
        acm = action_sync.wait(node, get.call_async(req), 8.).scene.allowed_collision_matrix
        fingers = [LEFT, RIGHT, 'arm_0_gripper_left_finger_prox_link',
                   'arm_0_gripper_right_finger_prox_link']
        for item in [name]+fingers:
            if item not in acm.entry_names:
                acm.entry_names.append(item)
                for row in acm.entry_values:
                    row.enabled.append(False)
                acm.entry_values.append(AllowedCollisionEntry(enabled=[False]*len(acm.entry_names)))
        index = acm.entry_names.index(name)
        for finger in fingers:
            j = acm.entry_names.index(finger)
            acm.entry_values[index].enabled[j] = acm.entry_values[j].enabled[index] = True
        scene = PlanningScene(is_diff=True, allowed_collision_matrix=acm)
        if not action_sync.wait(node, put.call_async(ApplyPlanningScene.Request(scene=scene)), 8.).success:
            raise RuntimeError('Contact policy rejected')
        return dict(object=name, allowed_links=fingers)
    finally:
        node.destroy_client(get)
        node.destroy_client(put)


def read():
    """一帧真值：{实体名: {position, yaw}}。"""
    return object_pose.read_entities()


def active_controllers(node):
    from controller_manager_msgs.srv import ListControllers
    client = node.create_client(ListControllers, '/a200_0000/controller_manager/list_controllers')
    try:
        if not client.wait_for_service(timeout_sec=5.):
            raise RuntimeError('P1: controller manager unavailable')
        res = action_sync.wait(node, client.call_async(ListControllers.Request()), 5.)
        active = sorted(c.name for c in res.controller if c.state == 'active')
        required = {'joint_state_broadcaster', 'platform_velocity_controller',
                    'arm_0_joint_trajectory_controller', 'arm_0_gripper_controller'}
        if not required <= set(active):
            raise RuntimeError('P1: required controllers inactive: '+str(required-set(active)))
        return active
    finally:
        node.destroy_client(client)


def remove_object(node, name):
    """把目标从规划场景里移除（合爪前必须做，否则末端被判成撞目标）。

    **但要留到最后一步再做**：一旦移除，规划器对这个物体就瞎了，运到起手位的那段摆臂会直接撞上去
    —— 10-23 晚实测：箱子被摆臂撞下台面，落到机器人前甲板上（world z 0.26）。
    """
    from moveit_msgs.msg import CollisionObject, PlanningScene
    from moveit_msgs.srv import ApplyPlanningScene
    client = node.create_client(ApplyPlanningScene, '/a200_0000/apply_planning_scene')
    if not client.wait_for_service(timeout_sec=10.0):
        return {'removed': False, 'reason': 'apply_planning_scene 不在'}
    event = CollisionObject()
    event.header.frame_id = 'map'
    event.id = name
    event.operation = CollisionObject.REMOVE
    scene = PlanningScene()
    scene.is_diff = True
    scene.world.collision_objects = [event]
    request = ApplyPlanningScene.Request()
    request.scene = scene
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=20.0)
    done = future.result()
    return {'removed': bool(done and done.success)}


def base_point(scene, name):
    """机器人 link -> base 系坐标（link 位置本身就是 base 口径，直接取）。"""
    return scene_geometry.local_point(scene, name)


def model_base(scene, name):
    """World -> model/base_footprint using full translation and quaternion rotation."""
    return scene_geometry.world_to_model(scene, ROBOT, name)


def jaw(scene):
    """(爪心, 物理开口宽度)，都在 base 系。"""
    left, right = base_point(scene, LEFT), base_point(scene, RIGHT)
    return (left + right) / 2.0, float(np.linalg.norm(left - right))
