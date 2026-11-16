"""A real SIGINT must deliver a measured hold before the command gate exits."""
import os
import signal
import subprocess
import sys
import time

import pytest

rclpy = pytest.importorskip('rclpy')
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from rclpy.qos import qos_profile_sensor_data


def test_sigint_delivers_measured_hold_before_exit(tmp_path):
    rclpy.init(domain_id=96)
    n = rclpy.create_node('test_stop_signal')
    log = (tmp_path/'bridge.log').open('w')
    env = dict(os.environ, ROS_DOMAIN_ID='96')
    child = subprocess.Popen([sys.executable, '-c',
        'from ffw_teleop.sharpa_vr_bridge import main; main()', '--ros-args',
        '-p', 'dry_run:=true', '-p', 'require_squeeze:=false'],
        env=env, stdout=log, stderr=subprocess.STDOUT)
    received = []
    feedback = n.create_publisher(JointState, '/joint_states', 10)
    poses = {}
    for side in ('left','right'):
        for kind, part in (('vr','raw'),('robot','state')):
            poses[(kind,side)] = n.create_publisher(PoseStamped, f'/vr/{part}/{side}/wrist_pose', 10)
    n.create_subscription(JointTrajectory, '/vr/preview/right/joint_trajectory',
                          lambda m: received.append(list(m.points[0].positions)), 10)
    ik = n.create_publisher(JointTrajectory, '/vr/ik/right/joint_trajectory', 10)
    def publish():
        names = [f'arm_{s}_joint{i}' for s in ('l','r') for i in range(1,8)]
        feedback.publish(JointState(name=names, position=[.1]*14))
        for (kind,side), pub in poses.items():
            msg = PoseStamped()
            msg.header.frame_id = 'base_link'
            msg.header.stamp = n.get_clock().now().to_msg()
            msg.pose.position.x = .5 if kind=='vr' else .4
            msg.pose.orientation.w = 1.
            pub.publish(msg)
    n.create_timer(.02, publish)
    client = n.create_client(SetBool, '/sharpa_vr_bridge/enable')
    try:
        assert client.wait_for_service(timeout_sec=8)
        enabled = False
        deadline = time.monotonic()+6
        while not enabled and time.monotonic()<deadline:
            for _ in range(10):
                rclpy.spin_once(n, timeout_sec=.02)
            future = client.call_async(SetBool.Request(data=True))
            rclpy.spin_until_future_complete(n, future, timeout_sec=2)
            enabled = future.done() and future.result().success
        assert enabled
        deadline = time.monotonic()+2
        while not received and time.monotonic()<deadline:
            ik.publish(JointTrajectory(joint_names=[f'arm_r_joint{i}' for i in range(1,8)],
                points=[JointTrajectoryPoint(positions=[.3]*7)]))
            rclpy.spin_once(n, timeout_sec=.02)
        assert received and received[-1]==[.3]*7
        child.send_signal(signal.SIGINT)
        deadline = time.monotonic()+3
        while (child.poll() is None or received[-1]!=[.1]*7) and time.monotonic()<deadline:
            rclpy.spin_once(n, timeout_sec=.02)
        assert child.poll()==0
        assert received[-1]==[.1]*7
    finally:
        if child.poll() is None:
            child.send_signal(signal.SIGINT)
            try:
                child.wait(timeout=3)
            except subprocess.TimeoutExpired:
                child.kill()
                child.wait()
        log.close()
        n.destroy_node()
        rclpy.shutdown()
