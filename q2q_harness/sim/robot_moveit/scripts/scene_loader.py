#!/usr/bin/env python3
"""把世界 SDF 里的静态 box 灌进 MoveIt 规划场景（frame = map，与第 3 步真值地图对齐）。

用法（VM 内，先 source ~/.q2q/step3_env.sh，move_group 在跑）：

    python3 scene_loader.py --world <q2q_warehouse.sdf>
    python3 scene_loader.py --world <q2q_warehouse.sdf> --test-box 0.55 0 1.15 0.3
    python3 scene_loader.py --remove-test-box

只认每个 model 的第一个 box collision；平面跳过。单次加载，包含动态 pick_bin 的初始位置。
move_group 重启后必须重新加载；物体移动后也必须更新，不能把初始 SDF 当实时场景。
"""

import argparse
import math
import time
import xml.etree.ElementTree as ET

import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.srv import ApplyPlanningScene
from rclpy.node import Node
import tf2_ros
from scipy.spatial.transform import Rotation

FRAME = 'map'
APPLY = '/a200_0000/apply_planning_scene'


def box_from_model(model):
    """<model> 里第一个 box collision → (name, pose[6], size[3])。"""
    pose = (model.findtext('pose') or '0 0 0 0 0 0').split()
    collision = model.find('link/collision/geometry/box/size')
    if collision is None:
        return None
    size = [float(v) for v in collision.text.split()]
    return model.get('name'), [float(v) for v in pose], size


def make_object(node, name, pose, size):
    obj = CollisionObject()
    obj.header.frame_id = FRAME
    obj.id = name
    obj.operation = CollisionObject.ADD
    primitive = SolidPrimitive()
    primitive.type = SolidPrimitive.BOX
    primitive.dimensions = size
    position = Pose()
    position.position.x, position.position.y, position.position.z = pose[0], pose[1], pose[2]
    roll, pitch, yaw = pose[3], pose[4], pose[5]
    position.orientation.z = math.sin(yaw / 2.0)
    position.orientation.w = math.cos(yaw / 2.0)
    if roll or pitch:
        node.get_logger().warn('%s 有 roll/pitch，本加载器只处理绕 z 的旋转' % name)
    obj.primitives = [primitive]
    obj.primitive_poses = [position]
    return obj


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--world', default='')
    parser.add_argument('--test-box', nargs=4, type=float, default=None,
                        metavar=('X', 'Y', 'Z', 'SIZE'))
    parser.add_argument('--remove-test-box', action='store_true')
    parser.add_argument('--remove', default='', help='从场景里移除同名几何（抓取目标在接近阶段要排除）')
    parser.add_argument('--skip-target', action='store_true')
    args = parser.parse_args()

    rclpy.init()
    node = Node('scene_loader', cli_args=['--ros-args', '-p', 'use_sim_time:=true',
                '-r', '/tf:=/a200_0000/tf', '-r', '/tf_static:=/a200_0000/tf_static'])
    buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(buffer, node)
    deadline = time.monotonic()+90.
    while not buffer.can_transform('base_footprint', FRAME, rclpy.time.Time()):
        if time.monotonic() > deadline:
            node.get_logger().error('Scene frame map unavailable; no scene applied')
            node.destroy_node()
            rclpy.shutdown()
            return 1
        rclpy.spin_once(node, timeout_sec=.1)
    client = node.create_client(ApplyPlanningScene, APPLY)
    if not client.wait_for_service(timeout_sec=15.0):
        node.get_logger().error('%s 不在：move_group 起了吗' % APPLY)
        return 1

    objects = []
    if args.world:
        for model in ET.parse(args.world).getroot().iter('model'):
            parsed = box_from_model(model)
            if parsed:
                if args.skip_target and parsed[0] == 'pick_bin':
                    continue
                objects.append(make_object(node, *parsed))
    if args.test_box:
        x, y, z, size = args.test_box
        objects.append(make_object(node, 'test_box', [x, y, z, 0, 0, 0], [size, size, size]))
    if args.remove:
        drop = CollisionObject()
        drop.header.frame_id = FRAME
        drop.id = args.remove
        drop.operation = CollisionObject.REMOVE
        objects.append(drop)
    if args.remove_test_box:
        obj = CollisionObject()
        obj.header.frame_id = FRAME
        obj.id = 'test_box'
        obj.operation = CollisionObject.REMOVE
        objects.append(obj)

    scene = PlanningScene()
    scene.is_diff = True
    # MoveIt's model frame is base_footprint. Resolve map explicitly at each
    # stationary manipulation station; storing initial world boxes once is stale
    # after the mobile base moves, and a cold MoveGroup may not know map yet.
    transform = buffer.lookup_transform('base_footprint', FRAME, rclpy.time.Time()).transform
    q = transform.rotation
    rotation = Rotation.from_quat([q.x, q.y, q.z, q.w])
    translation = transform.translation
    for obj in objects:
        obj.header.frame_id = 'base_footprint'
        for pose in obj.primitive_poses:
            p = pose.position
            xyz = rotation.apply([p.x, p.y, p.z])
            p.x, p.y, p.z = float(xyz[0]+translation.x), float(xyz[1]+translation.y), float(xyz[2]+translation.z)
            q = pose.orientation
            result = (rotation*Rotation.from_quat([q.x, q.y, q.z, q.w])).as_quat()
            q.x, q.y, q.z, q.w = map(float, result)
    if args.skip_target:
        drop = CollisionObject(id='pick_bin', operation=CollisionObject.REMOVE)
        objects.append(drop)
    scene.world.collision_objects = objects
    request = ApplyPlanningScene.Request()
    request.scene = scene
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=20.0)
    ok = bool(future.done() and future.result() and future.result().success)
    node.get_logger().info('应用 %d 个几何：%s' % (len(objects), 'success' if ok else 'FAILED'))
    node.destroy_node()
    rclpy.shutdown()
    return 0 if ok else 1


if __name__ == '__main__':
    raise SystemExit(main())
