"""Move both physical arms to the model reference before capturing VR anchors."""
import argparse
import math
import os
from pathlib import Path
import signal
import time

import rclpy
from builtin_interfaces.msg import Duration as DurationMsg
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from .stop_signals import flush_holds
from .teleop_recording import SessionRecorder
from .vr_start_pose import relaxed_arm_positions


class ArmInitializer(Node):
    def __init__(self, cancel_file=None, cancel_token='', minimum_duration=5., palms='inward'):
        super().__init__('initialize_vr_arms')
        self.cancel_file = Path(cancel_file) if cancel_file else None
        self.cancel_token = cancel_token
        self.minimum_duration = minimum_duration
        self.cancelled = False
        self.sent = False
        self.feedback = {}
        self.targets = relaxed_arm_positions(palms)
        self.recording = SessionRecorder('arm-initialization')
        self.pubs = {side: self.create_publisher(JointTrajectory,
            f'/arm_{side}_controller/joint_trajectory', 1) for side in ('l', 'r')}
        self.create_subscription(JointState, '/joint_states', self.on_joints, qos_profile_sensor_data)

    def on_joints(self, msg):
        now = time.monotonic()
        if len(msg.name) != len(msg.position):
            return
        for name, value in zip(msg.name, msg.position):
            if name in self.targets and math.isfinite(value):
                self.feedback[name] = (value, now)
        self.recording.record('measured_joints', msg, every=.05)

    def check_cancelled(self):
        if self.cancel_file:
            token = self.cancel_file.read_text().strip() if self.cancel_file.exists() else ''
            if token != self.cancel_token:
                self.cancelled = True
        if self.cancelled:
            raise InterruptedError('初期姿勢への移動を中止しました')

    def fresh(self):
        now = time.monotonic()
        return all(name in self.feedback and now-self.feedback[name][1] < .3 for name in self.targets)

    @staticmethod
    def names(side):
        return [f'arm_{side}_joint{i}' for i in range(1, 8)]

    def hold(self):
        if not self.sent:
            return
        now = time.monotonic()
        for side, pub in self.pubs.items():
            names = self.names(side)
            if not all(n in self.feedback and now-self.feedback[n][1] < .3 for n in names):
                continue
            pub.publish(JointTrajectory(joint_names=names, points=[JointTrajectoryPoint(
                positions=[self.feedback[n][0] for n in names],
                time_from_start=DurationMsg(nanosec=100_000_000))]))
        flush_holds(list(self.pubs.values()))
        self.recording.record('hold', {n: self.feedback[n][0] for n in self.targets if n in self.feedback})

    def run(self):
        deadline = time.monotonic()+10.
        while True:
            self.check_cancelled()
            rclpy.spin_once(self, timeout_sec=.02)
            if self.fresh() and all(p.get_subscription_count() for p in self.pubs.values()):
                break
            if time.monotonic() > deadline:
                raise RuntimeError('腕の実測値またはコントローラがありません。2番を確認してください')
        # Let the preceding measured-hold command settle before planning.
        settled_at = time.monotonic()+.3
        while time.monotonic() < settled_at:
            self.check_cancelled()
            rclpy.spin_once(self, timeout_sec=.02)
        if not self.fresh():
            raise RuntimeError('腕の実測値が途切れています')
        start = {n: self.feedback[n][0] for n in self.targets}
        maximum_delta = max(abs(start[n]-self.targets[n]) for n in self.targets)
        if maximum_delta < .02:
            self.recording.record('arrived', dict(max_error_rad=maximum_delta, already_at_reference=True))
            print('既にURDF初期姿勢です。この姿勢でVRの基準を登録します。', flush=True)
            return
        # Zero endpoint velocity and acceleration yield a quintic JTC spline.
        # Its peak speed is 1.875*distance/duration.
        duration = max(self.minimum_duration, maximum_delta*1.875/.4)
        ns = int(duration*1e9)
        stamp = (self.get_clock().now()+Duration(seconds=.15)).to_msg()
        print(f'URDF初期姿勢へ両腕を移動します（約{duration:.1f}秒）。停止は8番。', flush=True)
        self.recording.record('move_start', dict(initial=start, target=self.targets, duration=duration))
        for side, pub in self.pubs.items():
            names = self.names(side)
            trajectory = JointTrajectory(joint_names=names, points=[
                JointTrajectoryPoint(positions=[start[n] for n in names], velocities=[0.]*7, accelerations=[0.]*7),
                JointTrajectoryPoint(positions=[self.targets[n] for n in names], velocities=[0.]*7,
                    accelerations=[0.]*7, time_from_start=DurationMsg(sec=ns//1_000_000_000, nanosec=ns%1_000_000_000))])
            trajectory.header.stamp = stamp
            pub.publish(trajectory)
        self.sent = True
        started = time.monotonic()
        arrived_since = None
        while time.monotonic()-started < duration+5.:
            self.check_cancelled()
            rclpy.spin_once(self, timeout_sec=.02)
            if not self.fresh():
                raise RuntimeError('移動中に腕の実測値が途切れました。追従は開始しません')
            error = max(abs(self.feedback[n][0]-self.targets[n]) for n in self.targets)
            if time.monotonic()-started >= duration+.15 and error < .035:
                arrived_since = arrived_since or time.monotonic()
                if time.monotonic()-arrived_since >= .4:
                    self.recording.record('arrived', dict(max_error_rad=error))
                    self.hold()
                    print('URDF初期姿勢へ到着しました。この姿勢でVRの基準を登録します。', flush=True)
                    return
            else:
                arrived_since = None
        raise RuntimeError('初期姿勢へ到着しませんでした。追従は開始しません')


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--domain', type=int, default=0)
    parser.add_argument('--palms', choices=('inward', 'front'), default=os.environ.get('VR_PREVIEW_PALMS', 'inward'))
    options = parser.parse_args(args)
    rclpy.init(args=[], domain_id=options.domain, signal_handler_options=SignalHandlerOptions.NO)
    node = ArmInitializer(os.environ.get('AI_WORKER_OPERATION_FILE'), os.environ.get('AI_WORKER_OPERATION_TOKEN', ''),
                          palms=options.palms)
    previous = {}
    for sig in (signal.SIGINT, signal.SIGTERM):
        previous[sig] = signal.signal(sig, lambda *_: setattr(node, 'cancelled', True))
    status = 0
    try:
        node.run()
    except (RuntimeError, InterruptedError) as error:
        node.recording.record('stop', {'reason': str(error)})
        print(str(error), flush=True)
        node.hold()
        status = 130 if isinstance(error, InterruptedError) else 1
    finally:
        node.recording.close()
        node.destroy_node()
        rclpy.try_shutdown()
        for sig, handler in previous.items():
            signal.signal(sig, handler)
    return status


if __name__ == '__main__':
    raise SystemExit(main())
