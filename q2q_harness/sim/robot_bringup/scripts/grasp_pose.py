"""从「表面点 + 宽度」算抓取位姿；只做几何，不发运动、不依赖 ROS 消息包。

约定（教程 11 章）：
- 输入是**表面点**（深度图给的位置）和**宽度**（像素宽 × Z / fx 换算来的米），不是箱子中心；
- 接近方向 `approach` 是单位向量，指向物体；
- 输出位姿里，TCP 的 +z 轴沿 `approach`（指向物体），x 轴水平，y 轴由右手系定。

这是设计约定，不是本机已经标定过的夹爪 TCP 朝向——第 6 步真抓之前必须用实际末端位姿校准一次。
"""

import numpy as np


def width_from_pixels(width_px, depth, fx):
    """像素宽度 → 米。假设目标面近似垂直于视线，不垂直于视线时需要除 cos。"""
    if depth <= 0 or fx <= 0:
        raise ValueError('width_from_pixels: depth/fx 必须为正')
    return width_px * depth / fx


def rotation_from_approach(approach):
    """构造旋转：z 轴 = approach，x 轴水平（与世界 z 垂直），y = z × x。"""
    z = np.asarray(approach, dtype=float)
    norm = np.linalg.norm(z)
    if norm < 1e-6:
        raise ValueError('rotation_from_approach: approach 不能为零向量')
    z = z / norm
    world_z = np.array([0.0, 0.0, 1.0])
    x = np.cross(z, world_z)
    if np.linalg.norm(x) < 1e-6:          # approach 竖直时，改用世界 x 定水平轴
        x = np.cross(z, np.array([1.0, 0.0, 0.0]))
    x = x / np.linalg.norm(x)
    y = np.cross(z, x)
    return np.column_stack([x, y, z])      # 列为 x/y/z 轴在 base 下的方向


def grasp_pose(surface_point, width_m, approach, pre_grasp=0.10, insert=0.02):
    """返回 (pre_grasp_pose, grasp_pose, rotation)：位置是 base 系，旋转是 3×3。"""
    approach = np.asarray(approach, dtype=float)
    approach = approach / np.linalg.norm(approach)
    rotation = rotation_from_approach(approach)
    pre = np.asarray(surface_point, dtype=float) - approach * pre_grasp
    contact = np.asarray(surface_point, dtype=float) + approach * insert
    return {'pre_grasp': pre, 'grasp': contact, 'rotation': rotation, 'width_m': float(width_m)}


JAW_ROLL = np.array([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])  # 绕工具 z 自转 90°


def approach_quaternion(approach, jaw_horizontal=False):
    """接近方向 -> 末端四元数。`jaw_horizontal` 把默认的竖直开合转成水平开合（10-23 实测）。"""
    rotation = rotation_from_approach(approach)
    if jaw_horizontal:
        rotation = rotation @ JAW_ROLL
    return tuple(float(v) for v in rotation_to_quaternion(rotation))


def to_base(point, origin, yaw):
    """世界点 -> base 系坐标（只做平移 + 绕 z 旋转；base 的 z 就是 0）。"""
    dx, dy = point[0] - origin[0], point[1] - origin[1]
    cos, sin = np.cos(-yaw), np.sin(-yaw)
    return np.array([dx * cos - dy * sin, dx * sin + dy * cos, point[2] - origin[2]])


def rotation_to_quaternion(rotation):
    """Shepperd：任意旋转都能算（第 4 步那个 trace=0 的坑这里同样适用）。"""
    m = np.asarray(rotation, dtype=float)
    trace = m[0, 0] + m[1, 1] + m[2, 2]
    if trace > 0.0:
        s = np.sqrt(trace + 1.0) * 2.0
        return [(m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s,
                (m[1, 0] - m[0, 1]) / s, 0.25 * s]
    if m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
        return [0.25 * s, (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s,
                (m[2, 1] - m[1, 2]) / s]
    if m[1, 1] > m[2, 2]:
        s = np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
        return [(m[0, 1] + m[1, 0]) / s, 0.25 * s, (m[1, 2] + m[2, 1]) / s,
                (m[0, 2] - m[2, 0]) / s]
    s = np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
    return [(m[0, 2] + m[2, 0]) / s, (m[1, 2] + m[2, 1]) / s, 0.25 * s,
            (m[1, 0] - m[0, 1]) / s]
