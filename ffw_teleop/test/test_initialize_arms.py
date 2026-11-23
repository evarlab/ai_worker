import os
import json
import subprocess
import sys
import time

import pytest
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory

from ffw_teleop.vr_start_pose import relaxed_arm_positions


@pytest.mark.parametrize('cancel,custom,bad_lift', [(False,False,False),(True,False,False),(False,True,False),(True,True,False),(False,True,True)])
def test_initialization_waits_for_arrival_and_holds_when_cancelled(tmp_path, cancel, custom, bad_lift):
    # Only a synthetic robot in domain 93; no physical publishers or services.
    context = Context()
    rclpy.init(context=context, domain_id=93)
    node = rclpy.create_node('fake_initial_pose_robot', context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    target = relaxed_arm_positions()
    extra=[]
    if custom:
        target={name:value+.1 for name,value in target.items()}
        profile=tmp_path/'pose.json'
        profile.write_text(json.dumps(dict(arm_positions=target,joint_limits={n:[-3.14,3.14] for n in target},reference_lift_m=-.2)))
        extra=['--target-file',str(profile)]
    state = dict(target)
    state['arm_l_joint1'] = .2
    state['arm_r_joint1'] = -.2
    if custom:state['lift_joint']=-.15 if bad_lift else -.2
    initial = dict(state)
    messages = {'l': [], 'r': []}
    arrivals = {}
    pub = node.create_publisher(JointState, '/joint_states', 10)

    def trajectory(msg, side):
        messages[side].append(msg)
        if len(msg.points) == 2:
            duration = msg.points[-1].time_from_start
            arrivals[side] = time.monotonic()+duration.sec+duration.nanosec/1e9+.2

    for side in messages:
        node.create_subscription(JointTrajectory, f'/arm_{side}_controller/joint_trajectory',
            lambda msg, s=side: trajectory(msg, s), 10)

    def feedback():
        if not cancel:
            for side, when in arrivals.items():
                if time.monotonic() >= when:
                    for name in state:
                        if name.startswith(f'arm_{side}_'):
                            state[name] = target[name]
        msg = JointState(name=list(state), position=list(state.values()))
        msg.header.stamp = node.get_clock().now().to_msg()
        pub.publish(msg)

    node.create_timer(.02, feedback)
    marker = tmp_path/'stop-generation'
    marker.write_text('before')
    process = subprocess.Popen([sys.executable, '-m', 'ffw_teleop.initialize_arms', '--domain', '93', *extra],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
        env={**os.environ, 'HOME': str(tmp_path), 'AI_WORKER_OPERATION_FILE': str(marker),
             'AI_WORKER_OPERATION_TOKEN': 'before'})
    try:
        end = time.monotonic()+18
        while process.poll() is None and time.monotonic() < end:
            executor.spin_once(timeout_sec=.02)
            if cancel and all(messages.values()):
                marker.write_text('after')
        assert process.poll() is not None, 'Initializer did not finish'
        # Receive the final measured-hold publications before asserting.
        end = time.monotonic()+.3
        while time.monotonic() < end:
            executor.spin_once(timeout_sec=.02)
        output = process.communicate(timeout=2)[0]
        if bad_lift:
            assert process.returncode==1 and 'lift' in output,output
            assert not any(messages.values())
            return
        assert process.returncode == (130 if cancel else 0), output
        for side in messages:
            planned = next(msg for msg in messages[side] if len(msg.points) == 2)
            assert planned.header.stamp.sec == 0 and planned.header.stamp.nanosec == 0
            assert planned.points[-1].positions == pytest.approx([target[n] for n in planned.joint_names])
            held = messages[side][-1]
            expected = initial if cancel else target
            assert len(held.points) == 1
            assert held.points[0].positions == pytest.approx([expected[n] for n in held.joint_names])
        assert ('到着しました' in output) == (not cancel)
    finally:
        if process.poll() is None:
            process.kill()
            process.wait()
        executor.remove_node(node)
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown(context=context)
