"""Relay read-only physical inputs into an isolated model-only ROS domain."""
import argparse
from pathlib import Path
import time
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation

import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import String, Bool
from std_srvs.srv import SetBool
from trajectory_msgs.msg import JointTrajectory
from contact_glove_msgs.msg import Ergonomics
from .vr_pose_math import pose_matrix


class PreviewRecovery:
    """Retry only model-domain activation after fresh, continuous poses."""
    def __init__(self):
        self.poses = {}
        self.stable_since = {}
        self.active = self.requested = False
        self.retry_after = 0.

    def observe(self, side, matrix, now):
        previous = self.poses.get(side)
        if (previous is None or now-previous[1] > .3
                or np.linalg.norm(matrix[:3, 3]-previous[0][:3, 3]) > .3
                or Rotation.from_matrix(matrix[:3, :3] @ previous[0][:3, :3].T).magnitude() > 1.2):
            self.stable_since[side] = now
        self.poses[side] = (matrix, now)

    def status(self, active, now):
        if self.active and not active:
            self.requested = False
            self.retry_after = now+1.
        self.active = active

    def ready(self, now):
        return (not self.requested and now >= self.retry_after
                and all(s in self.poses and now-self.poses[s][1] < .2
                        and now-self.stable_since[s] >= 1. for s in ('left', 'right')))


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--domain', type=int, default=98)
    parser.add_argument('--source-domain', type=int, default=0)
    parser.add_argument('--urdf', type=Path, required=True)
    parser.add_argument('--start-pose', choices=('relaxed', 'measured'), default='relaxed',
                        help='relaxed: URDFの腕関節0（両腕を下げた姿勢）から開始')
    parser.add_argument('--palms', choices=('inward', 'front'), default='inward')
    options = parser.parse_args(args)
    if options.domain in (0, 30) or options.domain == options.source_domain:
        parser.error('モデル専用ドメインを指定してください（0/30は禁止）')
    contexts = [Context(), Context()]
    rclpy.init(context=contexts[0], domain_id=options.source_domain)
    rclpy.init(context=contexts[1], domain_id=options.domain)
    source = rclpy.create_node('vr_preview_input_only', context=contexts[0])
    model = rclpy.create_node('vr_preview_virtual_robot', context=contexts[1])
    executors = [SingleThreadedExecutor(context=c) for c in contexts]
    executors[0].add_node(source)
    executors[1].add_node(model)
    # There are deliberately no publishers in the physical context.
    names = [j.get('name') for j in ET.parse(options.urdf).getroot().findall('joint')
             if j.get('type') != 'fixed']
    positions, seen = {}, {'joints': 0., 'left': 0., 'right': 0.}
    arm_names = {f'arm_{s}_joint{i}' for s in ('l', 'r') for i in range(1, 8)}
    if options.start_pose == 'relaxed':
        positions = dict.fromkeys(names, 0.)
        if options.palms == 'inward':
            positions['arm_l_joint5'] = -np.pi/2
            positions['arm_r_joint5'] = np.pi/2
    joint_pub = model.create_publisher(JointState, '/joint_states', 10)
    recovery = PreviewRecovery()
    model.create_subscription(Bool, '/vr/active',
        lambda msg: recovery.status(msg.data, time.monotonic()), 10)

    def seed(msg):
        seen['joints'] = time.monotonic()
        for name, value in zip(msg.name, msg.position):
            is_arm = name in arm_names
            initial = 0. if options.start_pose == 'relaxed' and is_arm else float(value)
            if options.start_pose == 'relaxed' and options.palms == 'inward':
                if name == 'arm_l_joint5':
                    initial = -np.pi/2
                elif name == 'arm_r_joint5':
                    initial = np.pi/2
            positions.setdefault(name, initial)

    source.create_subscription(JointState, '/joint_states', seed, qos_profile_sensor_data)
    for side in ('left', 'right'):
        pose_pub = model.create_publisher(PoseStamped, f'/vr/raw/{side}/wrist_pose', 10)
        ergonomics_pub = model.create_publisher(Ergonomics, f'/contact_glove/{side}/ergonomics', 10)

        def pose(msg, pub=pose_pub, s=side):
            try:
                matrix = pose_matrix(msg.pose)
            except ValueError:
                return
            seen[s] = time.monotonic()
            recovery.observe(s, matrix, seen[s])
            pub.publish(msg)

        source.create_subscription(PoseStamped, f'/vr/raw/{side}/wrist_pose', pose, qos_profile_sensor_data)
        source.create_subscription(Ergonomics, f'/contact_glove/{side}/ergonomics',
                                   ergonomics_pub.publish, qos_profile_sensor_data)
    frame_pub = model.create_publisher(String, '/vr/input/reference_frame', 10)
    source.create_subscription(String, '/vr/input/reference_frame', frame_pub.publish, 10)

    def follow(msg):
        if msg.points:
            for name, value in zip(msg.joint_names, msg.points[-1].positions):
                if name in names:
                    positions[name] = float(value)

    for side in ('left', 'right'):
        for topic in (f'/vr/preview/{side}/joint_trajectory', f'/contactglove/{side}/preview'):
            model.create_subscription(JointTrajectory, topic, follow, 10)
    enable = model.create_client(SetBool, '/sharpa_vr_bridge/enable')
    pending, next_publish = None, 0.
    print('URDFプレビュー: 実機には出力しません。Ctrl+Cで終了。', flush=True)
    print(f'モデルの開始姿勢: {options.start_pose}, palms={options.palms}', flush=True)
    try:
        while all(c.ok() for c in contexts):
            for executor in executors:
                executor.spin_once(timeout_sec=.003)
            now = time.monotonic()
            if len(positions.keys() & set(names)) != len(names):
                continue
            if now >= next_publish and (options.start_pose == 'relaxed' or now-seen['joints'] < .5):
                msg = JointState(name=names, position=[positions[n] for n in names])
                msg.header.stamp = model.get_clock().now().to_msg()
                joint_pub.publish(msg)
                next_publish = now+.02
            if recovery.ready(now) and pending is None and enable.service_is_ready():
                pending = enable.call_async(SetBool.Request(data=True))
            if pending is not None and pending.done():
                result = pending.result()
                recovery.requested = result.success
                recovery.retry_after = now+1.
                print('プレビュー: '+result.message, flush=True)
                pending = None
    except KeyboardInterrupt:
        pass
    finally:
        for executor, node, context in zip(executors, (source, model), contexts):
            executor.remove_node(node)
            node.destroy_node()
            executor.shutdown()
            context.try_shutdown()
