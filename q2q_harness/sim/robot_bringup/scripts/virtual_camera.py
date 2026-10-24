#!/usr/bin/env python3
"""virtual_camera：用针孔投影把世界 SDF 的静态 box 渲染成合成 RGB/深度图。

这台 VM 的 Gazebo 渲染不可用（客户机 GL 是软件路径），所以第 5 步先用**明确标注的合成观测**
把「像素 → 相机系 → base」链路跑通；它不是视觉感知，验收时不算相机完成。

发布：`sensors/camera_0/{image, depth, camera_info}` + 静态 TF `base_footprint → camera_0_optical_frame`。
内参与教程 11 章一致：fx=fy=500、cx=320、cy=240、640×480。
"""

import math

import numpy as np

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from nav_msgs.msg import Odometry
from tf2_ros import StaticTransformBroadcaster

import camera_math

COLORS = {'bin': (200, 150, 60), 'shelf': (90, 110, 130), 'wall': (150, 150, 160),
          'workbench': (115, 105, 90), 'pick_bin': (60, 200, 90),
          'pick_stand': (115, 105, 90)}


class VirtualCamera(Node):
    def __init__(self):
        super().__init__('virtual_camera')
        self.declare_parameter('world_file', '')
        self.declare_parameter('mount_xyz', [0.45, 0.0, 0.32])
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('rate_hz', 3.0)
        self.declare_parameter('mount_pitch_deg', 0.0)
        self.declare_parameter('fx', 500.0)
        self.declare_parameter('fy', 500.0)

        self.fx = self.get_parameter('fx').value
        self.fy = self.get_parameter('fy').value
        self.w = int(self.get_parameter('width').value)
        self.h = int(self.get_parameter('height').value)
        self.cx, self.cy = self.w / 2.0, self.h / 2.0
        self.camera = np.array(self.get_parameter('mount_xyz').value, dtype=float)
        pitch = math.radians(float(self.get_parameter('mount_pitch_deg').value))
        # 俯仰：相机绕 base 的 y 轴向下转，再叠加原来的朝向约定
        tilt = np.array([[math.cos(pitch), 0.0, math.sin(pitch)],
                         [0.0, 1.0, 0.0],
                         [-math.sin(pitch), 0.0, math.cos(pitch)]])
        self.rot_ob = tilt @ camera_math.default_rotation()
        self.base_to_optical = self.rot_ob.T
        self.boxes = self._load_boxes(self.get_parameter('world_file').value)

        self.pub_image = self.create_publisher(Image, 'sensors/camera_0/image', 5)
        self.pub_depth = self.create_publisher(Image, 'sensors/camera_0/depth', 5)
        self.pub_info = self.create_publisher(CameraInfo, 'sensors/camera_0/camera_info', 5)
        self.base_pose = None
        self.create_subscription(Odometry, '/a200_0000/ground_truth/odom', self._on_odom, 10)
        self._publish_static_tf()
        rate = float(self.get_parameter('rate_hz').value)
        self.create_timer(1.0 / rate, self.render)
        self.get_logger().info('virtual_camera 就绪：%d 个 box，%.1f Hz' % (len(self.boxes), rate))

    def _load_boxes(self, path):
        import xml.etree.ElementTree as ET
        if not path:
            self.get_logger().warn('world_file 为空，只有背景')
            return []
        boxes = []
        for model in ET.parse(path).getroot().iter('model'):
            size = model.find('link/collision/geometry/box/size')
            pose = model.findtext('pose')
            if size is None or pose is None:
                continue
            name = model.get('name')
            center = [float(v) for v in pose.split()[:3]]
            kind = name.split('_')[0]
            boxes.append((name, center, [float(v) for v in size.text.split()],
                          COLORS.get(name) or COLORS.get(kind) or (120, 120, 120)))
        return boxes

    def _publish_static_tf(self):
        transform = TransformStamped()
        transform.header.frame_id = 'base_footprint'
        transform.child_frame_id = 'camera_0_optical_frame'
        transform.transform.translation.x = float(self.camera[0])
        transform.transform.translation.y = float(self.camera[1])
        transform.transform.translation.z = float(self.camera[2])
        # TF 要的是 optical→base 的旋转（vector_base = R · vector_optical），所以取转置
        quat = rotation_matrix_to_quaternion(self.rot_ob)
        transform.transform.rotation.x, transform.transform.rotation.y = quat[0], quat[1]
        transform.transform.rotation.z, transform.transform.rotation.w = quat[2], quat[3]
        StaticTransformBroadcaster(self).sendTransform(transform)

    def _on_odom(self, msg):
        self.base_pose = msg.pose.pose

    def render(self):
        depth = np.full((self.h, self.w), np.inf, dtype=np.float32)
        rgb = np.zeros((self.h, self.w, 3), dtype=np.uint8)
        if self.base_pose is None:                # 还没有底盘位姿，先不渲染
            return
        # 用真值里程计拿 base 在 map 里的位姿，不依赖 TF 订阅（命名空间里 TF 话题名多变）
        rot = quaternion_to_matrix(self.base_pose.orientation)
        trans = np.array([self.base_pose.position.x, self.base_pose.position.y,
                          self.base_pose.position.z])
        # 世界里的 box 是 map 系；机器人动了以后必须换到 base，不能假设 base==map
        # p_map = R·p_base + t，所以 p_base = R^T(p_map − t)；行向量写法见下
        origin_map = self.camera @ rot.T + trans
        for _, center, size, color in self.boxes:
            self._draw(center, size, color, depth, rgb, rot, trans, origin_map)
        stamp = self.get_clock().now().to_msg()
        self._publish(rgb, depth, stamp)

    def _draw(self, center, size, color, depth, rgb, rot, trans, origin_map):
        corners_base = (camera_math.box_corners(center, size) - trans) @ rot
        u, v, z = camera_math.project(corners_base, self.camera, self.fx, self.fy,
                                     self.cx, self.cy, self.rot_ob)
        if np.all(z <= 0.05):
            return
        u0, u1 = int(max(0, np.nanmin(u))), int(min(self.w - 1, np.nanmax(u)))
        v0, v1 = int(max(0, np.nanmin(v))), int(min(self.h - 1, np.nanmax(v)))
        if u1 <= u0 or v1 <= v0:
            return
        grid_u, grid_v = np.meshgrid(np.arange(u0, u1 + 1), np.arange(v0, v1 + 1))
        rays = np.stack([(grid_u - self.cx) / self.fx, (grid_v - self.cy) / self.fy,
                         np.ones_like(grid_u, dtype=float)], axis=-1)
        directions = rays @ self.rot_ob.T                 # optical→base（行向量）
        directions_map = directions @ rot.T               # base→map（行向量）
        hit = camera_math.ray_box(origin_map, directions_map, center, size)
        patch = depth[v0:v1 + 1, u0:u1 + 1]
        closer = hit < patch
        patch[closer] = hit[closer]
        rgb[v0:v1 + 1, u0:u1 + 1][closer] = color

    def _publish(self, rgb, depth, stamp):
        info = CameraInfo()
        info.header.stamp, info.header.frame_id = stamp, 'camera_0_optical_frame'
        info.width, info.height = self.w, self.h
        info.distortion_model = 'plumb_bob'
        info.d = [0.0] * 5
        info.k = [self.fx, 0.0, self.cx, 0.0, self.fy, self.cy, 0.0, 0.0, 1.0]
        info.p = [self.fx, 0.0, self.cx, 0.0, 0.0, self.fy, self.cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        self.pub_info.publish(info)

        image = Image()
        image.header.stamp, image.header.frame_id = stamp, 'camera_0_optical_frame'
        image.height, image.width, image.encoding = self.h, self.w, 'rgb8'
        image.step = self.w * 3
        image.data = rgb.tobytes()
        self.pub_image.publish(image)

        depth_msg = Image()
        depth_msg.header.stamp, depth_msg.header.frame_id = stamp, 'camera_0_optical_frame'
        depth_msg.height, depth_msg.width, depth_msg.encoding = self.h, self.w, '32FC1'
        depth_msg.step = self.w * 4
        depth_msg.data = np.where(np.isfinite(depth), depth, 0.0).astype(np.float32).tobytes()
        self.pub_depth.publish(depth_msg)


def quaternion_to_matrix(q):
    """四元数 (x,y,z,w) → 旋转矩阵（列 = 轴在父系的方向）。"""
    return np.array([
        [1 - 2 * (q.y ** 2 + q.z ** 2), 2 * (q.x * q.y - q.w * q.z), 2 * (q.x * q.z + q.w * q.y)],
        [2 * (q.x * q.y + q.w * q.z), 1 - 2 * (q.x ** 2 + q.z ** 2), 2 * (q.y * q.z - q.w * q.x)],
        [2 * (q.x * q.z - q.w * q.y), 2 * (q.y * q.z + q.w * q.x), 1 - 2 * (q.x ** 2 + q.y ** 2)],
    ])

def rotation_matrix_to_quaternion(matrix):
    """Shepperd 方法：四支分支，trace 为 0 的旋转（本例就是）也能算。"""
    m = np.asarray(matrix, dtype=float)
    trace = m[0, 0] + m[1, 1] + m[2, 2]
    if trace > 0.0:
        s = np.sqrt(trace + 1.0) * 2.0
        return [(m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s,
                (m[1, 0] - m[0, 1]) / s, 0.25 * s]
    if m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
        return [0.25 * s, (m[0, 1] + m[1, 0]) / s,
                (m[0, 2] + m[2, 0]) / s, (m[2, 1] - m[1, 2]) / s]
    if m[1, 1] > m[2, 2]:
        s = np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
        return [(m[0, 1] + m[1, 0]) / s, 0.25 * s,
                (m[1, 2] + m[2, 1]) / s, (m[0, 2] - m[2, 0]) / s]
    s = np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
    return [(m[0, 2] + m[2, 0]) / s, (m[1, 2] + m[2, 1]) / s,
            0.25 * s, (m[1, 0] - m[0, 1]) / s]


def main():
    rclpy.init()
    node = VirtualCamera()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
