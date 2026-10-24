#!/usr/bin/env python3
"""virtual_lidar.py —— 几何射线激光（替代渲染激光，2026-10-21 起）

**为什么要它**：这台 VM 的 GPU 渲染拿不到深度（`glxinfo` 显示 SVGA3D `Accelerated: no`，
只给 GL 4.1；ogre2 起得来线程，但 `gpu_lidar` 的 720 束里只有 8 束有值、且全等于 24.999 = range_max）。
结果 slam 只能扫出自由空间、0 个障碍格。换成 VMSVGA/llvmpipe/EGL/降低束数都试过，都不行。

**它做什么**：读世界 SDF 里的长方体（墙 / 货架 / 料箱），按机器人当前位姿做 2D 射线求交，
按 15 Hz 发标准 `sensor_msgs/LaserScan`。走的是**物理几何**而不是渲染，所以和渲染后端无关。

**边界（论文里必须写明）**：
  * 只支持轴对齐长方体（我们的世界全是 box + 一个地面平面），不支持网格模型；
  * 不含噪声、不含反射率；所有 box 高度投影到二维平面，包括低于激光的料箱；
  * 当前 odom 由 Gazebo 真值插件发布，和 world 原点一致；不能直接改回轮式估计而不处理坐标/漂移；
  * 上层一行不用改 —— Nav2 / slam_toolbox 拿到的就是普通 LaserScan。

用法（bringup.launch.py 自动起，也可手动）：
  ros2 run python3 virtual_lidar.py --world <world.sdf> --topic /a200_0000/sensors/lidar2d_0/scan
"""

import argparse
import math
import xml.etree.ElementTree as ET

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
import tf2_ros

DEFAULT_SAMPLES = 720
DEFAULT_RATE = 15.0
DEFAULT_RANGE_MAX = 25.0
DEFAULT_RANGE_MIN = 0.05


def load_boxes(path):
    """从 SDF 里取出所有轴对齐长方体：[(xmin, xmax, ymin, ymax), ...]

    只认 <model><pose>x y z …</pose> 与 <geometry><box><size>。
    我们的 q2q_warehouse.sdf 全是这种形状；换个带网格的世界，这个函数就要跟着改。
    地面用的是 <plane>，这里显式跳过（水平射线本来就不该打地面）；
    同一个模型里 visual 和 collision 各有一份 box，按坐标去重。
    """
    boxes = set()
    root = ET.parse(path).getroot()
    for world in root.findall('world'):
        for model in world.findall('model'):
            pose = model.find('pose')
            if pose is None:
                continue
            values = [float(v) for v in pose.text.split()]
            mx, my = values[0], values[1]
            if len(values) > 5 and any(abs(v) > 1e-6 for v in values[3:6]):
                continue                        # 带旋转的模型不支持
            if model.find('.//plane') is not None:
                continue                        # 地面/天花板之类的平面
            for size in model.iter('size'):
                dims = [float(v) for v in size.text.split()]
                if len(dims) < 2:
                    continue
                hx, hy = dims[0] / 2.0, dims[1] / 2.0
                boxes.add((round(mx - hx, 3), round(mx + hx, 3),
                           round(my - hy, 3), round(my + hy, 3)))
    return sorted(boxes)


def ray_box_distance(origin, direction, box):
    """一条射线对一个 AABB 的入射距离；不相交返回 None。标准 slab 法。"""
    ox, oy = origin
    dx, dy = direction
    xmin, xmax, ymin, ymax = box
    # 平行于 slab 且原点在外侧时永远不相交。
    if abs(dx) < 1e-9 and not xmin <= ox <= xmax:
        return None
    if abs(dy) < 1e-9 and not ymin <= oy <= ymax:
        return None
    tx_min, tx_max = (-1e9, 1e9) if abs(dx) < 1e-9 else ((xmin - ox) / dx, (xmax - ox) / dx)
    if tx_min > tx_max:
        tx_min, tx_max = tx_max, tx_min
    ty_min, ty_max = (-1e9, 1e9) if abs(dy) < 1e-9 else ((ymin - oy) / dy, (ymax - oy) / dy)
    if ty_min > ty_max:
        ty_min, ty_max = ty_max, ty_min
    enter = max(tx_min, ty_min)
    leave = min(tx_max, ty_max)
    if leave < 0.0 or enter > leave:
        return None
    return enter if enter > 0.0 else None


class VirtualLidar(Node):

    def __init__(self, world, topic, frame, samples, rate, range_min, range_max):
        super().__init__('virtual_lidar')
        self.boxes = load_boxes(world)
        self.frame = frame
        self.samples = samples
        self.range_min = range_min
        self.range_max = range_max
        self.rate = rate
        self.angle_min = -math.pi
        self.angle_increment = 2.0 * math.pi / samples
        self.buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        tf2_ros.TransformListener(self.buffer, self)
        self.publisher = self.create_publisher(LaserScan, topic, 10)
        self.create_timer(1.0 / rate, self.publish_scan)
        self.misses = 0
        self.get_logger().info('虚拟激光就绪：%d 个长方体，%d 束，%.0f Hz，话题 %s'
                               % (len(self.boxes), samples, rate, topic))

    def lidar_pose(self):
        """odom 帧里激光的位姿：射线原点 + 朝向。拿不到 TF 就返回 None（这一帧跳过）。"""
        try:
            transform: TransformStamped = self.buffer.lookup_transform(
                'odom', self.frame, Time(), Duration(seconds=0.2))
        except Exception:                        # noqa: BLE001 —— TF 还没来是正常情况
            self.misses += 1
            if self.misses % 150 == 1:           # 15 Hz 下约每 10 s 提醒一次
                self.get_logger().warn('查不到 odom→%s 的 TF（累计 %d 次）；'
                                       '下一步该查：TF 是否发在 /a200_0000/tf（要不要加 /tf 重映射）'
                                       % (self.frame, self.misses))
            return None
        position = transform.transform.translation
        quaternion = transform.transform.rotation
        yaw = math.atan2(2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
                         1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z))
        return (position.x, position.y), yaw

    def scan_ranges(self, origin, yaw):
        ranges = []
        for index in range(self.samples):
            angle = yaw + self.angle_min + index * self.angle_increment
            direction = (math.cos(angle), math.sin(angle))
            nearest = None
            for box in self.boxes:
                distance = ray_box_distance(origin, direction, box)
                if distance is not None and (nearest is None or distance < nearest):
                    nearest = distance
            if nearest is None or nearest > self.range_max or nearest < self.range_min:
                ranges.append(float('inf'))
            else:
                ranges.append(float(nearest))
        return ranges

    def publish_scan(self):
        pose = self.lidar_pose()
        if pose is None:
            return
        origin, yaw = pose
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = self.frame
        scan.angle_min = self.angle_min
        scan.angle_max = self.angle_min + (self.samples - 1) * self.angle_increment
        scan.angle_increment = self.angle_increment
        scan.time_increment = 0.0
        scan.scan_time = 1.0 / self.rate
        scan.range_min = self.range_min
        scan.range_max = self.range_max
        scan.ranges = self.scan_ranges(origin, yaw)
        self.publisher.publish(scan)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--world', required=True, help='世界 SDF 绝对路径')
    parser.add_argument('--topic', default='/a200_0000/sensors/lidar2d_0/scan')
    parser.add_argument('--frame', default='lidar2d_0_laser')
    parser.add_argument('--samples', type=int, default=DEFAULT_SAMPLES)
    parser.add_argument('--rate', type=float, default=DEFAULT_RATE)
    parser.add_argument('--range-min', type=float, default=DEFAULT_RANGE_MIN)
    parser.add_argument('--range-max', type=float, default=DEFAULT_RANGE_MAX)
    # parse_known_args：`--ros-args -r /tf:=...` 这类参数要原样留给 rclpy.init()
    args, _ = parser.parse_known_args()

    rclpy.init()
    node = VirtualLidar(args.world, args.topic, args.frame, args.samples,
                        args.rate, args.range_min, args.range_max)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
