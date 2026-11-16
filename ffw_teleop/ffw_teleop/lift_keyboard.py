"""Standalone arrow-key lift control. Does not start recording or arm teleop."""
import argparse
import math
import os
import select
import signal
import sys
import termios
import time
import tty

import rclpy
from builtin_interfaces.msg import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class ArrowKeys:
    def __init__(self):
        self.buffer = b''

    def feed(self, data):
        self.buffer += data
        keys = []
        while self.buffer:
            if self.buffer.startswith(b'\x1b'):
                if len(self.buffer) < 3:
                    break
                sequence, self.buffer = self.buffer[:3], self.buffer[3:]
                if sequence in (b'\x1b[A', b'\x1bOA'):
                    keys.append('up')
                elif sequence in (b'\x1b[B', b'\x1bOB'):
                    keys.append('down')
            else:
                key, self.buffer = self.buffer[:1], self.buffer[1:]
                if key == b' ':
                    keys.append('hold')
                elif key in (b'q', b'Q', b'\x03'):
                    keys.append('quit')
        return keys


class LiftKeyboard(Node):
    def __init__(self, step=.01, duration=.25, dry_run=False):
        super().__init__('standalone_lift_keyboard')
        self.step, self.duration, self.dry_run = step, duration, dry_run
        self.feedback = None
        self.last_command = -math.inf
        self.pub = self.create_publisher(JointTrajectory, '/lift_controller/joint_trajectory', 1)
        self.create_subscription(JointState, '/joint_states', self.on_joints, qos_profile_sensor_data)

    def on_joints(self, msg):
        if 'lift_joint' in msg.name:
            index = msg.name.index('lift_joint')
            if index < len(msg.position) and math.isfinite(msg.position[index]):
                self.feedback = (float(msg.position[index]), time.monotonic())

    def command(self, direction):
        now = time.monotonic()
        if self.feedback is None or now-self.feedback[1] > .5:
            print('昇降の実測値がありません。実機本体とjoint_state_broadcasterを確認してください。', flush=True)
            return None
        if direction and now-self.last_command < .08:
            return None
        # Always step from fresh measured height, never accumulate queued travel
        # while a key is held. SG2 URDF: positive Z is up, range [-0.5, 0] m.
        target = min(0., max(-.5, self.feedback[0]+direction*self.step))
        ns = int((self.duration if direction else .1)*1e9)
        msg = JointTrajectory(joint_names=['lift_joint'], points=[JointTrajectoryPoint(
            positions=[target], velocities=[0.], time_from_start=Duration(sec=ns//1_000_000_000,
                                                                         nanosec=ns%1_000_000_000))])
        msg.header.stamp = self.get_clock().now().to_msg()
        if not self.dry_run:
            if self.pub.get_subscription_count() == 0:
                print('lift_controllerの受信先がありません。', flush=True)
                return None
            self.pub.publish(msg)
        self.last_command = now
        print(f'昇降 {self.feedback[0]:+.3f} → {target:+.3f} m'+(' [dry-run]' if self.dry_run else ''), flush=True)
        return target


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--step', type=float, default=.01, help='1キーの移動量[m]')
    parser.add_argument('--duration', type=float, default=.25)
    parser.add_argument('--domain', type=int, default=int(os.environ.get('ROS_DOMAIN_ID', '0')))
    parser.add_argument('--dry-run', action='store_true')
    options = parser.parse_args(args)
    if not math.isfinite(options.step) or not 0 < options.step <= .05 or not math.isfinite(options.duration) or options.duration < .1:
        parser.error('stepは0〜0.05m、durationは0.1秒以上')
    if not sys.stdin.isatty():
        parser.error('キーボードを使えるターミナルで起動してください')
    rclpy.init(args=[], domain_id=options.domain, signal_handler_options=SignalHandlerOptions.NO)
    node = LiftKeyboard(options.step, options.duration, options.dry_run)
    stopping = [False]
    previous = {sig: signal.signal(sig, lambda *_: stopping.__setitem__(0, True))
                for sig in (signal.SIGINT, signal.SIGTERM)}
    attributes = termios.tcgetattr(sys.stdin)
    keys = ArrowKeys()
    print('昇降のみ: ↑ 上げる / ↓ 下げる / Space 保持 / q・Ctrl+C 終了。記録は別コマンドです。', flush=True)
    try:
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok() and not stopping[0]:
            rclpy.spin_once(node, timeout_sec=.01)
            if select.select([sys.stdin], [], [], .02)[0]:
                data = os.read(sys.stdin.fileno(), 64)
                if not data:
                    break
                for key in keys.feed(data):
                    if key == 'quit':
                        stopping[0] = True
                    else:
                        node.command({'up': 1, 'down': -1, 'hold': 0}[key])
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, attributes)
        node.command(0)
        from .stop_signals import flush_holds
        flush_holds([node.pub])
        node.destroy_node()
        rclpy.try_shutdown()
        for sig, handler in previous.items():
            signal.signal(sig, handler)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
