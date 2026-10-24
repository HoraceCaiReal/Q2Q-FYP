#!/usr/bin/env python3
"""实时 2D 视图（Tk Canvas，不依赖 matplotlib / OpenGL）。

这台 VM 的软件 GL 下，Gazebo 的 3D 视口是空白的、RViz 的 Qt xcb 插件也起不来；
Tk 直接画 2D 是唯一稳定可用的可视化。画面的坐标系就是 map 帧，与真值 odom 一致。

- 灰色轮廓 = 世界文件里的静态长方体（货架 / 工位 / 台面）
- 品红方块 = 目标物体 pick_bin（Gazebo 真值，后台线程每 0.5 s 问一次）
- 绿色圆点 + 短线段 = 底盘真实位姿与朝向；绿色细线 = 走过的轨迹
- 左上状态条 = episode / 状态 / x / y / yaw / 锁 / 已用时间
"""

import math
import re
import subprocess
import threading
import time
import tkinter as tk
import xml.etree.ElementTree as ET

import rclpy
from nav_msgs.msg import Odometry
from q2q_interfaces.msg import TaskState
from rclpy.node import Node

WORLD = '/home/ros/.q2q/runs/current/world.sdf'
X0, X1, Y0, Y1 = -1.6, 2.4, -2.0, 2.0
W, H, PAD = 780, 660, 28


def footprints():
    out = []
    try:
        root = ET.parse(WORLD).getroot()
    except Exception:
        return out
    for model in root.findall('world/model'):
        if (model.findtext('static') or '').strip() != 'true':
            continue
        pose = [float(v) for v in (model.findtext('pose') or '0 0 0 0 0 0').split()[:6]]
        size = model.findtext('link/collision/geometry/box/size')
        if size:
            out.append((pose[0], pose[1], float(size.split()[0]), float(size.split()[1]), pose[5]))
    return out


class Viewer(Node):
    def __init__(self):
        super().__init__('demo_view')
        self.state, self.episode, self.locked = 'IDLE', '-', False
        self.pose = (0.0, 0.0, 0.0)
        self.create_subscription(Odometry, '/a200_0000/ground_truth/odom', self.on_odom, 10)
        self.create_subscription(TaskState, '/q2q/mid/task_state', self.on_state, 20)

    def on_odom(self, msg):
        p = msg.pose.pose
        yaw = math.atan2(2.0 * (p.orientation.w * p.orientation.z + p.orientation.x * p.orientation.y),
                         1.0 - 2.0 * (p.orientation.y ** 2 + p.orientation.z ** 2))
        self.pose = (p.position.x, p.position.y, yaw)

    def on_state(self, msg):
        self.state, self.episode = msg.state, msg.episode_id
        self.locked = msg.base_command_locked


def poll_box(holder):
    while True:
        try:
            text = subprocess.run(['ign', 'model', '-m', 'pick_bin', '-p'], capture_output=True,
                                  text=True, timeout=3).stdout
            # 输出里第一对方括号是实体 id、第二/三对是列标题，第四对才是位姿
            match = re.findall(r'\[\s*(-?\d+\.?\d*)\s+(-?\d+\.?\d*)\s+(-?\d+\.?\d*)\s*\]', text)
            if match:
                holder['xy'] = (float(match[-2][0]), float(match[-2][1]))
        except Exception:
            pass
        time.sleep(0.5)


def main():
    blocks = footprints()
    rclpy.init()
    node = Viewer()
    holder = {'xy': None}
    threading.Thread(target=poll_box, args=(holder,), daemon=True).start()
    started = time.time()
    trail = []

    root = tk.Tk()
    root.title('Q2Q step-7 - live top view (map frame)')
    canvas = tk.Canvas(root, width=W, height=H, bg='white', highlightthickness=0)
    canvas.pack()
    sx = (W - 2 * PAD) / (X1 - X0)
    sy = (H - 2 * PAD) / (Y1 - Y0)
    s = min(sx, sy)

    def px(x, y):
        return PAD + (x - X0) * s, H - PAD - (y - Y0) * s

    for cx, cy, w, d, yaw in blocks:
        pts = []
        for dx, dy in ((-w / 2, -d / 2), (w / 2, -d / 2), (w / 2, d / 2), (-w / 2, d / 2)):
            rx = cx + dx * math.cos(yaw) - dy * math.sin(yaw)
            ry = cy + dx * math.sin(yaw) + dy * math.cos(yaw)
            pts.extend(px(rx, ry))
        canvas.create_polygon(pts, outline='#9aa0a6', fill='', width=1)
    ox, oy = px(0, 0)
    canvas.create_line(ox, PAD, ox, H - PAD, fill='#e0e0e0')
    canvas.create_line(PAD, oy, W - PAD, oy, fill='#e0e0e0')

    while True:
        rclpy.spin_once(node, timeout_sec=0.02)
        x, y, yaw = node.pose
        trail.append((x, y))
        if len(trail) > 1500:
            del trail[:-1500]
        canvas.delete('dyn')
        if len(trail) > 1:
            canvas.create_line([c for p in trail for c in px(*p)], fill='#a5d6a7', width=1, tags='dyn')
        if holder['xy'] is not None:
            bx, by = px(*holder['xy'])
            canvas.create_rectangle(bx - 5, by - 5, bx + 5, by + 5, fill='#d81b60', outline='', tags='dyn')
        rx, ry = px(x, y)
        canvas.create_oval(rx - 7, ry - 7, rx + 7, ry + 7, outline='#1b5e20', width=2, tags='dyn')
        canvas.create_line(rx, ry, *px(x + 0.28 * math.cos(yaw), y + 0.28 * math.sin(yaw)),
                           fill='#1b5e20', width=3, tags='dyn')
        canvas.delete('hud')
        canvas.create_text(12, 10, anchor='nw', tags='hud', font=('Courier', 10), fill='#263238',
                           text='episode %s   elapsed %5.1f s\nSTATE   %-12s  locked=%s\n'
                                'x=%+.3f  y=%+.3f  yaw=%+.3f rad'
                                % (node.episode[-6:] if node.episode != '-' else '-',
                                   time.time() - started, node.state, node.locked, x, y, yaw))
        root.update()
        time.sleep(0.05)


if __name__ == '__main__':
    main()
