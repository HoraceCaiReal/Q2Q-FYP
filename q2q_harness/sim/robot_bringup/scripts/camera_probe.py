#!/usr/bin/env python3
"""验证「像素 → 相机系 → base」：从合成图里找橙色料箱，用针孔模型 + TF 算回 base，和 SDF 真值对比。

用法（VM 内，先 source ~/.q2q/step3_env.sh）：
    python3 camera_probe.py --truth 1.6 0.2 0.15

不做检测模型：只按颜色找连通块的质心，当作「检测器输出的像素」。误差只反映坐标链，不反映感知能力。
"""

import argparse
import json
import time

import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener

BIN_RGB = (200, 150, 60)
TOLERANCE = 40


class Probe(Node):
    def __init__(self):
        super().__init__('camera_probe')
        self.rgb = self.depth = self.info = None
        self.create_subscription(Image, '/a200_0000/sensors/camera_0/image', self._on_rgb, 5)
        self.create_subscription(Image, '/a200_0000/sensors/camera_0/depth', self._on_depth, 5)
        self.create_subscription(CameraInfo, '/a200_0000/sensors/camera_0/camera_info', self._on_info, 5)
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)

    def _on_rgb(self, msg):
        self.rgb = msg

    def _on_depth(self, msg):
        self.depth = msg

    def _on_info(self, msg):
        self.info = msg

    def ready(self):
        return all(x is not None for x in (self.rgb, self.depth, self.info))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--truth', nargs=3, type=float, required=True)
    args = parser.parse_args()

    rclpy.init()
    node = Probe()
    deadline = time.monotonic() + 20.0
    while not node.ready() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    if not node.ready():
        print(json.dumps({'error': '拿不到 image/depth/camera_info'}, ensure_ascii=False))
        return 1

    rgb = np.frombuffer(node.rgb.data, dtype=np.uint8).reshape(node.rgb.height, node.rgb.width, 3)
    depth = np.frombuffer(node.depth.data, dtype=np.float32).reshape(node.depth.height,
                                                                    node.depth.width)
    info = node.info
    fx, fy = info.k[0], info.k[4]
    cx, cy = info.k[2], info.k[5]

    mask = np.all(np.abs(rgb.astype(int) - np.array(BIN_RGB)) < TOLERANCE, axis=2)
    if not mask.any():
        print(json.dumps({'error': '图里没有橙色料箱', 'pixels': int(mask.sum())}, ensure_ascii=False))
        return 1
    rows, cols = np.nonzero(mask)
    u, v = float(cols.mean()), float(rows.mean())
    z = float(np.median(depth[rows, cols]))
    if z <= 0.0:
        print(json.dumps({'error': '深度无效', 'depth': z}, ensure_ascii=False))
        return 1

    time.sleep(0.2)
    rclpy.spin_once(node, timeout_sec=0.2)
    transform = node.buffer.lookup_transform('base_footprint', info.header.frame_id,
                                             rclpy.time.Time())
    t = transform.transform.translation
    q = transform.transform.rotation
    x_axis = np.array([1 - 2 * (q.y * q.y + q.z * q.z), 2 * (q.x * q.y + q.w * q.z),
                       2 * (q.x * q.z - q.w * q.y)])
    y_axis = np.array([2 * (q.x * q.y - q.w * q.z), 1 - 2 * (q.x * q.x + q.z * q.z),
                       2 * (q.y * q.z + q.w * q.x)])
    z_axis = np.array([2 * (q.x * q.z + q.w * q.y), 2 * (q.y * q.z - q.w * q.x),
                       1 - 2 * (q.x * q.x + q.y * q.y)])
    p_cam = np.array([(u - cx) * z / fx, (v - cy) * z / fy, z])
    p_base = np.array([t.x, t.y, t.z]) + p_cam[0] * x_axis + p_cam[1] * y_axis + p_cam[2] * z_axis
    truth = np.array(args.truth)
    print(json.dumps({
        'pixel': [round(u, 1), round(v, 1)], 'depth_m': round(z, 4),
        'p_base': [round(float(x), 4) for x in p_base],
        'truth': args.truth,
        'error_m': round(float(np.linalg.norm(p_base - truth)), 4),
        'frame': info.header.frame_id, 'pixels_used': int(mask.sum()),
    }, ensure_ascii=False))
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
