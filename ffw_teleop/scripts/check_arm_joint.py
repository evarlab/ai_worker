#!/usr/bin/env python3
"""Bounded one-degree wrist probe; records JTC reference and actual joint feedback."""
import math
import signal
import time

import rclpy
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from control_msgs.msg import JointTrajectoryControllerState
from controller_manager_msgs.srv import ListControllers
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration as DurationMsg

JOINT = 'arm_r_joint7'
CONTROLLER = 'arm_r_controller'
STEP = 0.02


def main():
    stop = [False]
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stop.__setitem__(0, True))
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('check_arm_joint')
    measured, controller_rows = [], []
    def joint_callback(msg):
        if JOINT in msg.name:
            value = msg.position[msg.name.index(JOINT)]
            if math.isfinite(value):
                measured.append((time.monotonic(), value))
    def state_callback(msg):
        if JOINT in msg.joint_names:
            index = msg.joint_names.index(JOINT)
            if len(msg.reference.positions)>index and len(msg.feedback.positions)>index:
                controller_rows.append((msg.reference.positions[index], msg.feedback.positions[index]))
    node.create_subscription(JointState, '/joint_states', joint_callback, qos_profile_sensor_data)
    node.create_subscription(JointTrajectoryControllerState, f'/{CONTROLLER}/controller_state', state_callback, 10)
    pub = node.create_publisher(JointTrajectory, f'/{CONTROLLER}/joint_trajectory', 1)
    client = node.create_client(ListControllers, '/controller_manager/list_controllers')
    sent = False
    try:
        if not client.wait_for_service(timeout_sec=15):
            raise RuntimeError('Controller manager is unavailable')
        deadline = time.monotonic()+20
        while time.monotonic()<deadline and not stop[0]:
            future = client.call_async(ListControllers.Request())
            rclpy.spin_until_future_complete(node, future, timeout_sec=2)
            active = future.done() and any(c.name==CONTROLLER and c.state=='active' for c in future.result().controller)
            if active and measured and time.monotonic()-measured[-1][0]<.3:
                break
            rclpy.spin_once(node, timeout_sec=.1)
        else:
            raise RuntimeError('Need fresh wrist feedback and active arm controller')
        initial = measured[-1][1]
        if not -1.5<initial<1.5:
            raise RuntimeError('Wrist is near its limit; no probe sent')
        goal = initial+STEP
        print(f'{JOINT}: initial={initial:.6f}, probe={goal:.6f} rad', flush=True)
        points = [JointTrajectoryPoint(positions=[q], time_from_start=DurationMsg(sec=t))
                  for q,t in ((goal,2),(goal,3),(initial,5))]
        pub.publish(JointTrajectory(joint_names=[JOINT], points=points))
        sent = True
        started = time.monotonic()
        while time.monotonic()-started<7 and not stop[0]:
            rclpy.spin_once(node, timeout_sec=.01)
            if not measured or time.monotonic()-measured[-1][0]>.3:
                raise RuntimeError('Joint feedback lost; probe cancelled')
            if abs(measured[-1][1]-initial)>.06:
                raise RuntimeError('Unexpected motion; probe cancelled')
        span = max(v for _,v in measured)-min(v for _,v in measured)
        reference_span = max((x[0] for x in controller_rows), default=initial)-min((x[0] for x in controller_rows), default=initial)
        print(f'measured range={span:.6f} rad, JTC reference range={reference_span:.6f} rad', flush=True)
        print(f'final error from initial={measured[-1][1]-initial:.6f} rad, interrupted={stop[0]}', flush=True)
        if not stop[0] and span<.01:
            raise RuntimeError('Controller received a target, but actual wrist movement was not confirmed')
    finally:
        if sent and measured and time.monotonic()-measured[-1][0]<.3:
            hold = JointTrajectoryPoint(positions=[measured[-1][1]], time_from_start=DurationMsg(nanosec=50000000))
            pub.publish(JointTrajectory(joint_names=[JOINT], points=[hold]))
            pub.wait_for_all_acked(Duration(seconds=.2))
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
