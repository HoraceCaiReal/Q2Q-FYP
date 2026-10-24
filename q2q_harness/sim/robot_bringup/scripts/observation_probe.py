#!/usr/bin/env python3
"""observation_probe：订阅一条 GraspObservation，打印契约字段并与真值对账。

用法（VM 内，先 source ~/.q2q/step3_env.sh）：
    python3 observation_probe.py --truth 1.6 0.2 0.15 --size 0.4
"""

import argparse
import json
import time

import numpy as np

import rclpy
from rclpy.node import Node
from q2q_interfaces.msg import GraspObservation
import object_pose
import scene_geometry


class Probe(Node):
    def __init__(self):
        super().__init__('observation_probe')
        self.latest = None
        self.create_subscription(GraspObservation, '/a200_0000/perception/grasp_observation',
                                 self._on, 5)

    def _on(self, msg):
        self.latest = msg


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--truth', nargs=3, type=float, required=True)
    parser.add_argument('--size', type=float, default=0.4)
    parser.add_argument('--truth-frame', choices=['world', 'base_footprint'], default='world')
    args = parser.parse_args()

    rclpy.init()
    node = Probe()
    deadline = time.monotonic() + 20.0
    while node.latest is None and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
    if node.latest is None:
        print(json.dumps({'error': '没收到 GraspObservation'}, ensure_ascii=False))
        return 1

    msg = node.latest
    pose = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
    truth = np.array(args.truth)
    if msg.header.frame_id != 'base_footprint':
        raise RuntimeError('Unexpected observation frame: '+msg.header.frame_id)
    if args.truth_frame == 'world':
        scene = object_pose.read_entities()
        scene['_truth'] = dict(position=truth)
        truth = scene_geometry.world_to_model(scene, 'a200_0000/robot', '_truth')
    print(json.dumps({
        'stamp_ns': msg.header.stamp.sec * 10 ** 9 + msg.header.stamp.nanosec,
        'frame_id': msg.header.frame_id,
        'truth_input_frame': args.truth_frame,
        'truth_center_base': truth.tolist(),
        'target_id': msg.target_id,
        'valid': msg.valid,
        'reason': msg.reason,
        'width_m': round(msg.width_m, 4),
        'width_error_m': round(abs(msg.width_m - args.size), 4),
        'confidence': round(msg.confidence, 4),
        'pose_position': [round(float(v), 4) for v in pose],
        'distance_to_truth_center_m': round(float(np.linalg.norm(pose - truth)), 4),
        'scene_version': msg.scene_version,
        'quaternion_xyzw': [round(msg.pose.orientation.x, 4), round(msg.pose.orientation.y, 4),
                            round(msg.pose.orientation.z, 4), round(msg.pose.orientation.w, 4)],
    }, ensure_ascii=False))
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
