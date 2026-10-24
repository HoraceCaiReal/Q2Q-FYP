#!/usr/bin/env python3
"""observation_node：把「图里的橙色块」变成契约化的抓取观测并发布。

发布 `/a200_0000/perception/grasp_observation`（`q2q_interfaces/msg/GraspObservation`）。

诚实边界：
- 检测器就是颜色阈值 + 质心，不是模型；`confidence` 用「块内像素一致度」当代理，不是模型置信度；
- `target_id` 用位置分桶生成，物体不动时稳定，动了会换 id（真跟踪器才解决跨帧关联）；
- `scene_version` 没有生产者，先发空串。
"""

import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener

import grasp_pose
from q2q_interfaces.msg import GraspObservation

BIN_RGB = (60, 200, 90)   # 抓取工位上的 pick_bin（颜色由 virtual_camera 的 COLORS['pick'] 定义）
TOLERANCE = 40
MIN_PIXELS = 200
GRIPPER_MAX_M = 0.96        # URDF 里夹爪关节上限


class ObservationNode(Node):
    def __init__(self):
        super().__init__('observation_node')
        self.rgb = self.depth = self.info = None
        self.create_subscription(Image, '/a200_0000/sensors/camera_0/image', self._on_rgb, 5)
        self.create_subscription(Image, '/a200_0000/sensors/camera_0/depth', self._on_depth, 5)
        self.create_subscription(CameraInfo, '/a200_0000/sensors/camera_0/camera_info',
                                 self._on_info, 5)
        self.pub = self.create_publisher(GraspObservation,
                                         '/a200_0000/perception/grasp_observation', 5)
        self.buffer = Buffer()
        TransformListener(self.buffer, self)
        self.create_timer(0.5, self.detect)
        self.get_logger().info('observation_node 就绪')

    def _on_rgb(self, msg):
        self.rgb = msg

    def _on_depth(self, msg):
        self.depth = msg

    def _on_info(self, msg):
        self.info = msg

    def detect(self):
        if self.rgb is None or self.depth is None or self.info is None:
            return
        rgb = np.frombuffer(self.rgb.data, np.uint8).reshape(self.rgb.height, self.rgb.width, 3)
        mask = np.all(np.abs(rgb.astype(int) - np.array(BIN_RGB)) < TOLERANCE, axis=2)
        observation = GraspObservation()
        observation.header.stamp = self.rgb.header.stamp
        observation.header.frame_id = 'base_footprint'
        observation.scene_version = ''
        if mask.sum() < MIN_PIXELS:
            self._reject(observation, 'no_target_in_view')
            return
        depth = np.frombuffer(self.depth.data, np.float32).reshape(self.depth.height,
                                                                   self.depth.width)
        rows, cols = np.nonzero(mask)
        z = float(np.median(depth[rows, cols]))
        if z <= 0.0 or not np.isfinite(z):
            self._reject(observation, 'invalid_depth')
            return
        fx, fy = self.info.k[0], self.info.k[4]
        cx, cy = self.info.k[2], self.info.k[5]
        u, v = float(cols.mean()), float(rows.mean())
        try:
            origin, x_axis, y_axis, z_axis = self._camera_axes()
        except Exception as error:                    # TF 还没到
            self._reject(observation, 'no_tf:%s' % error)
            return
        p_cam = np.array([(u - cx) * z / fx, (v - cy) * z / fy, z])
        surface = origin + p_cam[0] * x_axis + p_cam[1] * y_axis + p_cam[2] * z_axis
        # 宽度用 3D 范围，不用像素宽：斜看时像素宽会系统性偏小（实测 0.25 vs 0.40）。
        # 做法：把掩码像素反投影到 base，再沿夹爪闭合轴（x 轴）取跨度。
        step = max(1, len(rows) // 4000)
        uu, vv = cols[::step].astype(float), rows[::step].astype(float)
        zz = depth[rows[::step], cols[::step]]
        ok = zz > 0.0
        uu, vv, zz = uu[ok], vv[ok], zz[ok]
        points = (origin + ((uu - cx) * zz / fx)[:, None] * x_axis[None, :]
                  + ((vv - cy) * zz / fy)[:, None] * y_axis[None, :]
                  + zz[:, None] * z_axis[None, :])
        closing = np.cross(z_axis, np.array([0.0, 0.0, 1.0]))   # 相机轴向的水平垂线
        if np.linalg.norm(closing) < 1e-6:
            closing = np.array([0.0, 1.0, 0.0])
        closing = closing / np.linalg.norm(closing)
        extent = points @ closing
        width_m = float(extent.max() - extent.min())
        # 没有真值可用：接近方向取「相机看向该点的视线方向」，即从相机侧接近。
        # 真实系统应由物体朝向估计给出，这里明确写成启发式。
        approach = surface - origin
        approach = approach / np.linalg.norm(approach)
        pose = grasp_pose.grasp_pose(surface, width_m, approach)
        quat = grasp_pose.rotation_to_quaternion(pose['rotation'])

        if width_m > GRIPPER_MAX_M:
            self._reject(observation, 'width_over_gripper')
            return
        observation.target_id = 'bin_%d_%d' % (int(surface[0] * 10), int(surface[1] * 10))
        observation.pose.position.x, observation.pose.position.y = float(pose['grasp'][0]), float(pose['grasp'][1])
        observation.pose.position.z = float(pose['grasp'][2])
        observation.pose.orientation.x, observation.pose.orientation.y = float(quat[0]), float(quat[1])
        observation.pose.orientation.z, observation.pose.orientation.w = float(quat[2]), float(quat[3])
        observation.width_m = float(width_m)
        observation.confidence = float(mask.sum()) / float(mask.sum() + MIN_PIXELS)
        observation.valid = True
        observation.reason = 'ok'
        self.pub.publish(observation)

    def _reject(self, observation, reason):
        observation.valid = False
        observation.reason = reason
        observation.confidence = 0.0
        self.pub.publish(observation)

    def _camera_axes(self):
        t = self.buffer.lookup_transform('base_footprint', 'camera_0_optical_frame',
                                         rclpy.time.Time())
        p, q = t.transform.translation, t.transform.rotation
        x = np.array([1 - 2 * (q.y ** 2 + q.z ** 2), 2 * (q.x * q.y + q.w * q.z),
                      2 * (q.x * q.z - q.w * q.y)])
        y = np.array([2 * (q.x * q.y - q.w * q.z), 1 - 2 * (q.x ** 2 + q.z ** 2),
                      2 * (q.y * q.z + q.w * q.x)])
        z = np.array([2 * (q.x * q.z + q.w * q.y), 2 * (q.y * q.z - q.w * q.x),
                      1 - 2 * (q.x ** 2 + q.y ** 2)])
        return np.array([p.x, p.y, p.z]), x, y, z


def main():
    rclpy.init()
    node = ObservationNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
