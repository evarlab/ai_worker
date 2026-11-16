#!/usr/bin/env python3
"""Exercise the real Cyclo solver with synthetic feedback in an isolated ROS domain."""
import argparse
import collections
import copy
from pathlib import Path
import time
import xml.etree.ElementTree as ET

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rcl_interfaces.srv import GetParameters
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32
from std_srvs.srv import SetBool
from trajectory_msgs.msg import JointTrajectory


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--urdf', required=True)
    parser.add_argument('--domain', type=int, default=97)
    args = parser.parse_args()
    if args.domain in (0, 30):
        parser.error('Use an isolated domain, not the physical robot domain 0 or 30')
    rclpy.init(domain_id=args.domain)
    node = rclpy.create_node('check_vr_ik')
    parameters = node.create_client(GetParameters, '/sharpa_vr_bridge/get_parameters')
    if not parameters.wait_for_service(timeout_sec=15):
        raise RuntimeError('Start sharpa_vr.launch.py dry_run:=true in the isolated domain')
    future = parameters.call_async(GetParameters.Request(names=['dry_run']))
    rclpy.spin_until_future_complete(node, future, timeout_sec=5)
    if not future.done() or not future.result().values[0].bool_value:
        raise RuntimeError('Refusing to run unless the bridge confirms dry_run=true')
    joints = [j for j in ET.parse(args.urdf).getroot().findall('joint') if j.get('type') != 'fixed']
    names = [j.get('name') for j in joints]
    positions = []
    for j in joints:
        name = j.get('name')
        value = 0.0
        if name.startswith(('arm_l_joint', 'arm_r_joint')):
            i = int(name[-1])
            value = {1: .3, 2: .25 if '_l_' in name else -.25, 4: -1.3}.get(i, 0.0)
        limit = j.find('limit')
        if j.get('type') != 'continuous' and limit is not None:
            value = max(float(limit.get('lower', value)), min(float(limit.get('upper', value)), value))
        positions.append(value)
    state_pub = node.create_publisher(JointState, '/joint_states', 10)
    pose_pubs, grip_pubs, initial, targets, fk = {}, {}, {}, collections.defaultdict(list), {}
    for side in ('left', 'right'):
        def pose_callback(msg, side=side):
            initial.setdefault(side, copy.deepcopy(msg))
            fk[side] = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        def target_callback(msg, side=side):
            if msg.points:
                targets[side].append((time.monotonic(), np.array(msg.points[0].positions)))
        node.create_subscription(PoseStamped, f'/vr/state/{side}/wrist_pose', pose_callback, qos_profile_sensor_data)
        node.create_subscription(JointTrajectory, f'/vr/preview/{side}/joint_trajectory', target_callback, 10)
        pose_pubs[side] = node.create_publisher(PoseStamped, f'/vr/raw/{side}/wrist_pose', 10)
        grip_pubs[side] = node.create_publisher(Float32, f'/vr/input/{side}/squeeze', 10)
    activation = node.create_client(SetBool, '/sharpa_vr_bridge/enable')
    phase = [False]
    def publish():
        state_pub.publish(JointState(name=names, position=positions, velocity=[0.0]*len(names)))
        for side, reference in initial.items():
            msg = copy.deepcopy(reference)
            msg.header.stamp = node.get_clock().now().to_msg()
            if phase[0]:
                msg.pose.position.x += .03
                msg.pose.position.y += .01 if side == 'left' else -.01
            pose_pubs[side].publish(msg)
            grip_pubs[side].publish(Float32(data=1.0))
    node.create_timer(.02, publish)
    try:
        end = time.monotonic()+20
        while len(initial)<2 and time.monotonic()<end:
            rclpy.spin_once(node, timeout_sec=.02)
        if len(initial)<2:
            raise RuntimeError('No initial forward-kinematics poses')
        end = time.monotonic()+1
        while time.monotonic()<end:
            rclpy.spin_once(node, timeout_sec=.02)
        future = activation.call_async(SetBool.Request(data=True))
        rclpy.spin_until_future_complete(node, future, timeout_sec=5)
        if not future.done() or not future.result().success:
            raise RuntimeError(f'Activation failed: {future.result()}')
        start = time.monotonic()
        while time.monotonic()-start<6:
            rclpy.spin_once(node, timeout_sec=.01)
        before = {s: targets[s][-1][1].copy() for s in ('left','right') if targets[s]}
        phase[0] = True
        print('Applying 3 cm forward / 1 cm lateral wrist reference', flush=True)
        while time.monotonic()-start<15:
            rclpy.spin_once(node, timeout_sec=.01)
        passed = True
        for side in ('left','right'):
            if side not in before or not targets[side]:
                print(side, 'NO ARM OUTPUT', flush=True)
                passed = False
                continue
            change = float(np.max(np.abs(targets[side][-1][1]-before[side])))
            expected = np.array([initial[side].pose.position.x+.03,
                                 initial[side].pose.position.y+(.01 if side=='left' else -.01),
                                 initial[side].pose.position.z])
            print(side, 'commands', len(targets[side]), 'max joint change rad', round(change,6),
                  'predicted wrist error m', round(float(np.linalg.norm(fk[side]-expected)),6), flush=True)
            passed &= change > .001
        if not passed:
            raise RuntimeError('Cartesian reference did not produce a measurable joint target change')
    finally:
        if activation.service_is_ready():
            future = activation.call_async(SetBool.Request(data=False))
            rclpy.spin_until_future_complete(node, future, timeout_sec=2)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
