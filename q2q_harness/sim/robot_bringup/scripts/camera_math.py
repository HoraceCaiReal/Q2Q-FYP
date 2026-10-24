"""相机数学：投影、光线-长方体求交、像素 → base 坐标。

约定（教程 11 章）：optical 系 x 右、y 下、z 前；body 系 x 前、y 左、z 上。
两者之间只差一个固定旋转，本文件把旋转写死在 BASE_TO_OPTICAL 里，换安装朝向时改这一处。
"""

import numpy as np

# base(前左上) → optical(右下前) 的旋转：x_opt=-y_base, y_opt=-z_base, z_opt=+x_base
BASE_TO_OPTICAL = np.array([[0.0, -1.0, 0.0],
                            [0.0, 0.0, -1.0],
                            [1.0, 0.0, 0.0]])


def box_corners(center, size):
    """轴对齐长方体的 8 个角点（base 系）。"""
    half = np.asarray(size, dtype=float) / 2.0
    signs = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)])
    return np.asarray(center, dtype=float) + signs * half


def default_rotation():
    """optical→base 的默认旋转（相机朝前、不俯仰）。"""
    return BASE_TO_OPTICAL.T


def project(points_base, camera_base, fx, fy, cx, cy, rot_ob=None):
    """base 点 → (u, v, 深度)；rot_ob 是 optical→base 的旋转，可带俯仰。"""
    rot = default_rotation() if rot_ob is None else np.asarray(rot_ob, dtype=float)
    rel = np.asarray(points_base, dtype=float) - np.asarray(camera_base, dtype=float)
    q = rel @ rot                      # R^T · rel（R = optical→base）
    z = q[:, 2]
    safe = np.where(np.abs(z) < 1e-6, np.nan, z)
    return fx * q[:, 0] / safe + cx, fy * q[:, 1] / safe + cy, z


def ray_box(origin, directions, center, size):
    """向量化的光线-长方体求交；directions 已归一化或不必归一，返回交点距离（失败为 inf）。"""
    lower = np.asarray(center) - np.asarray(size) / 2.0
    upper = np.asarray(center) + np.asarray(size) / 2.0
    with np.errstate(divide='ignore', invalid='ignore'):
        t1 = (lower - origin) / directions
        t2 = (upper - origin) / directions
    tmin = np.nanmax(np.minimum(t1, t2), axis=-1)
    tmax = np.nanmin(np.maximum(t1, t2), axis=-1)
    hit = (tmax >= np.maximum(tmin, 0.0))
    return np.where(hit, tmin, np.inf)


def pixel_to_base(u, v, depth, camera_base, fx, fy, cx, cy):
    """针孔模型 + 固定旋转：像素 + 光学深度 → base 系点。"""
    q = np.array([(u - cx) * depth / fx, (v - cy) * depth / fy, depth])
    return np.asarray(camera_base, dtype=float) + BASE_TO_OPTICAL.T @ q
