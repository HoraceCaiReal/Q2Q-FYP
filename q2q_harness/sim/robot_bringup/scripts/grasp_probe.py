#!/usr/bin/env python3
"""抓取位姿探针：合成图 → 像素/深度 → 表面点 + 像素宽度 → 抓取位姿 → 与真值对账。

用法（VM 内，先 source ~/.q2q/step3_env.sh）：
    python3 grasp_probe.py --truth 1.6 0.2 0.15 --size 0.4

只做几何验证：不规划、不运动。宽度用「像素宽 × Z / fx」换算，并和箱子真实边长比。
"""

import argparse
import json
import time

import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener

import grasp_pose

BIN_RGB = (200, 150, 60)
TOLERANCE = 40


class Probe(Node):
    def __init__(self):
        super().__init__('grasp_probe')
        self.rgb = self.depth = self.info = None
        self.create_subscription(Image, '/a200_0000/sensors/camera_0/image', self._on_rgb, 5)
        self.create_subscription(Image, '/a200_0000/sensors/camera_0/depth', self._on_depth, 5)
        self.create_subscription(CameraInfo, '/a200_0000/sensors/camera_0/camera_info',
                                 self._on_info, 5)
        self.buffer = Buffer()
        TransformListener(self.buffer, self)

    def _on_rgb(self, msg):
        self.rgb = msg

    def _on_depth(self, msg):
        self.depth = msg

    def _on_info(self, msg):
        self.info = msg

    def ready(self):
        return all(x is not None for x in (self.rgb, self.depth, self.info))

    def camera_axes(self):
        t = self.buffer.lookup_transform('base_footprint', 'camera_0_optical_frame',
                                         rclpy.time.Time())
        p = t.transform.translation
        q = t.transform.rotation
        x = np.array([1 - 2 * (q.y ** 2 + q.z ** 2), 2 * (q.x * q.y + q.w * q.z),
                      2 * (q.x * q.z - q.w * q.y)])
        y = np.array([2 * (q.x * q.y - q.w * q.z), 1 - 2 * (q.x ** 2 + q.z ** 2),
                      2 * (q.y * q.z + q.w * q.x)])
        z = np.array([2 * (q.x * q.z + q.w * q.y), 2 * (q.y * q.z - q.w * q.x),
                      1 - 2 * (q.x ** 2 + q.y ** 2)])
        return np.array([p.x, p.y, p.z]), x, y, z


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--truth', nargs=3, type=float, required=True)
    parser.add_argument('--size', type=float, default=0.4)
    args = parser.parse_args()

    rclpy.init()
    node = Probe()
    deadline = time.monotonic() + 20.0
    while not node.ready() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    if not node.ready():
        print(json.dumps({'error': '拿不到 image/depth/camera_info'}, ensure_ascii=False))
        return 1

    rgb = np.frombuffer(node.rgb.data, np.uint8).reshape(node.rgb.height, node.rgb.width, 3)
    depth = np.frombuffer(node.depth.data, np.float32).reshape(node.depth.height,
                                                               node.depth.width)
    fx, fy, cx, cy = node.info.k[0], node.info.k[4], node.info.k[2], node.info.k[5]

    mask = np.all(np.abs(rgb.astype(int) - np.array(BIN_RGB)) < TOLERANCE, axis=2)
    if not mask.any():
        print(json.dumps({'error': '图里没有橙色料箱'}, ensure_ascii=False))
        return 1
    rows, cols = np.nonzero(mask)
    u, v = float(cols.mean()), float(rows.mean())
    z = float(np.median(depth[rows, cols]))
    width_px = float(cols.max() - cols.min() + 1)

    time.sleep(0.2)
    rclpy.spin_once(node, timeout_sec=0.2)
    origin, x_axis, y_axis, z_axis = node.camera_axes()
    p_cam = np.array([(u - cx) * z / fx, (v - cy) * z / fy, z])
    surface = origin + p_cam[0] * x_axis + p_cam[1] * y_axis + p_cam[2] * z_axis

    width_m = grasp_pose.width_from_pixels(width_px, z, fx)
    approach = (np.array(args.truth) - surface)
    approach = approach / np.linalg.norm(approach)   # 从表面指向箱子中心
    pose = grasp_pose.grasp_pose(surface, width_m, approach)
    quat = grasp_pose.rotation_to_quaternion(pose['rotation'])

    print(json.dumps({
        'pixel': [round(u, 1), round(v, 1)], 'depth_m': round(z, 4),
        'surface_base': [round(float(x), 4) for x in surface],
        'width_px': round(width_px, 1),
        'width_m': round(width_m, 4), 'width_truth_m': args.size,
        'width_error_m': round(abs(width_m - args.size), 4),
        'pre_grasp': [round(float(x), 4) for x in pose['pre_grasp']],
        'grasp': [round(float(x), 4) for x in pose['grasp']],
        'quaternion_xyzw': [round(float(x), 4) for x in quat],
        'approach': [round(float(x), 4) for x in approach],
    }, ensure_ascii=False))
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
