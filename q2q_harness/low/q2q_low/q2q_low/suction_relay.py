#!/usr/bin/env python3
"""suction_relay：**运动学搬运**（第 7 步方案 2，用户选的）。

做一件事：收到「抱住」后，用后台线程不断把物体在 Gazebo 里的位姿设成「末端位姿 + 固定偏移」，
收到「松开」就停。所以物体是被**搬动**的，不参与夹持力学。

当前选择运动学近似以验证调度。早期关于DetachableJoint在本机不可用的判断已被撤回，
不作为当前方案的技术前提；该物理插件尚未验收。

**必须标注**：这是运动学搬运（kinematic carry），不是夹持、也不是物理吸附。记录里写
`grasp_mode=suction_approx`，答辩口径不得写成「夹住了」。

接口（ROS）：`/q2q/low/suction`  std_msgs/msg/Bool  true=抱住  false=松开
实现：TF与真值里程计得到末端世界位置，后台线程通过常驻Fortress客户端发出位姿命令。
50mm世界的显式支撑插件清零动量并保持位姿；detach或500ms失联后恢复自由运动。
`/q2q/low/suction_active`报告支撑切换确认，实际跟随及落点仍独立查真值。
"""

import re
import subprocess
import threading
import time
import select
from pathlib import Path
from ament_index_python.packages import get_package_prefix

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool
import tf2_ros
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, DurabilityPolicy

POSE_INFO = '/world/%s/pose/info'
SET_POSE = '/world/%s/set_pose'


class KinematicCarry(Node):
    def __init__(self):
        super().__init__('suction_relay')
        self.declare_parameter('world', 'q2q_warehouse')
        self.declare_parameter('model', 'pick_bin')
        self.declare_parameter('tip_link', 'arm_0_gripper_left_finger_dist_link')
        self.declare_parameter('period', 0.05)
        self.world = self.get_parameter('world').value
        self.model = self.get_parameter('model').value
        self.tip_link = self.get_parameter('tip_link').value
        self.period = float(self.get_parameter('period').value)
        helper = Path(get_package_prefix('robot_sim_control'))/'lib/robot_sim_control/carry_pose_client'
        self.client = subprocess.Popen([str(helper), '/q2q/kinematic_carry/set_pose', self.model],
                                       stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                       text=True, bufsize=1)
        self.base, self.offset, self.busy, self.thread = None, None, False, None
        self.active = False
        self.status = self.create_publisher(Bool, '/q2q/low/suction_active',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.status.publish(Bool(data=False))
        self.buffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.buffer, self)
        self.create_subscription(Odometry, '/a200_0000/ground_truth/odom', self.on_odom, 10)
        self.create_subscription(Bool, '/q2q/low/suction', self.on_command, 1)
        self.get_logger().warn('运动学搬运就绪（不是夹持、不是物理吸附）：%s ← %s'
                               % (self.model, self.tip_link))

    def on_odom(self, msg):
        pose = msg.pose.pose
        self.base = (pose.position.x, pose.position.y, pose.position.z,
                     pose.orientation.x, pose.orientation.y,
                     pose.orientation.z, pose.orientation.w)

    @staticmethod
    def apply_pose(pose7, point):
        """base 系里的点 → map：先按四元数旋转，再平移。"""
        x, y, z, qx, qy, qz, qw = pose7
        px, py, pz = point
        tx = 2.0 * (qy * pz - qz * py)
        ty = 2.0 * (qz * px - qx * pz)
        tz = 2.0 * (qx * py - qy * px)
        return (x + px + qw * tx + (qy * tz - qz * ty),
                y + py + qw * ty + (qz * tx - qx * tz),
                z + pz + qw * tz + (qx * ty - qy * tx))

    def tip_world(self):
        if self.base is None:
            return None
        try:
            tf = self.buffer.lookup_transform('base_footprint', self.tip_link, rclpy.time.Time())
        except Exception:
            return None
        return self.apply_pose(self.base, (tf.transform.translation.x,
                                           tf.transform.translation.y,
                                           tf.transform.translation.z))

    def model_position(self):
        """读一次物体世界位置（只在抱住瞬间用一次，不进控制环）。"""
        output = subprocess.run(['ign', 'topic', '-e', '-t', POSE_INFO % self.world, '-n', '1'],
                                capture_output=True, text=True, timeout=20).stdout
        for block in output.split('pose {')[1:]:
            if 'name: "%s"' % self.model in block:
                found = re.search(r'position \{(.*?)\}', block, re.S)
                if found:
                    # Protobuf text omits zero-valued coordinates (especially y=0).
                    values = dict(re.findall(r'\b([xyz]):\s*([-+0-9.eE]+)', found.group(1)))
                    return tuple(float(values.get(axis, 0.0)) for axis in 'xyz')
        return None

    def on_command(self, msg):
        if not msg.data:
            self.busy = False
            self.get_logger().warn('运动学搬运：松开')
            return
        tip, box = self.tip_world(), self.model_position()
        if tip is None or box is None:
            self.get_logger().error('抱不住：拿不到末端或物体位置；下一步该查：TF 与 /pose/info')
            return
        self.offset = tuple(box[i] - tip[i] for i in range(3))
        self.busy = True
        self.get_logger().warn('运动学搬运：开始跟手（偏移 %.3f %.3f %.3f）' % self.offset)
        if self.thread is None:
            self.thread = threading.Thread(target=self.loop, daemon=True)
            self.thread.start()

    def loop(self):
        while rclpy.ok():
            if not self.busy:
                if self.active:
                    self.client.stdin.write('detach\n')
                    self.client.stdin.flush()
                    if select.select([self.client.stdout], [], [], 1.)[0]:
                        self.client.stdout.readline()
                    self.active = False
                    self.status.publish(Bool(data=False))
                time.sleep(0.1)
                continue
            tip = self.tip_world()
            if tip is not None and self.offset is not None:
                x, y, z = (tip[i] + self.offset[i] for i in range(3))
                try:
                    self.client.stdin.write('%.8f %.8f %.8f\n' % (x, y, z))
                    self.client.stdin.flush()
                    if not select.select([self.client.stdout], [], [], 1.)[0]:
                        raise RuntimeError('carry service response timed out')
                    if self.client.stdout.readline().strip() != '1':
                        raise RuntimeError('carry service did not acknowledge pose')
                    if not self.active and self.busy:
                        self.active = True
                        self.status.publish(Bool(data=True))
                except Exception as error:
                    self.get_logger().error('set_pose 失败：%s' % error)
            time.sleep(self.period)


def main():
    rclpy.init()
    node = KinematicCarry()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        node.busy = False
        node.client.terminate()
        node.client.wait(timeout=3)
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
