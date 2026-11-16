"""Stream ContactGlove3 Pro flexion to existing Sharpa trajectory controllers."""
import math
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

from .contactglove_mapping import ABDUCTION, FLEXION, abduction_targets, ergonomics_frame, slew, targets
from .stop_signals import flush_holds, spin_until_stopped


class ContactGloveSharpa(Node):
    def __init__(self):
        super().__init__('contactglove_sharpa')
        defaults = dict(host='127.0.0.1', port=25791, local_port=0, hands='right',
                        controlled_fingers='all',
                        input_mode='sdk', glove_namespace='/contact_glove',
                        auto_enable=False, clamp_targets=True,
                        include_abduction=False,
                        left_abduction_gains=[1.0]*5, right_abduction_gains=[1.0]*5,
                        left_abduction_offsets=[0.0]*5, right_abduction_offsets=[0.0]*5,
                        left_glove_id='', right_glove_id='', rate=30.0,
                        timeout=0.3, max_frame_age=0.5, max_speed=0.5,
                        horizon=0.1, dry_run=True,
                        left_gains=[1.0] * 15, right_gains=[1.0] * 15,
                        left_offsets=[0.0] * 15, right_offsets=[0.0] * 15,
                        left_lower=[0.0] * 15, right_lower=[0.0] * 15,
                        left_upper=[0.35] * 15, right_upper=[0.35] * 15)
        self.p = {k: self.declare_parameter(k, v).value for k, v in defaults.items()}
        if self.p['hands'] not in ('left', 'right', 'both'):
            raise ValueError('hands must be left, right or both')
        self.sides = ('left', 'right') if self.p['hands'] == 'both' else (self.p['hands'],)
        if self.p['controlled_fingers'] not in ('all', 'four_fingers'):
            raise ValueError('controlled_fingers must be all or four_fingers')
        self.mapping = FLEXION + ABDUCTION if self.p['include_abduction'] else FLEXION
        self.indices = [i for i, (name, _, _) in enumerate(self.mapping)
                        if self.p['controlled_fingers'] == 'all' or not name.startswith('thumb')]
        for key in ('rate', 'timeout', 'max_frame_age', 'horizon'):
            if not math.isfinite(self.p[key]) or self.p[key] <= 0:
                raise ValueError(f'{key} must be finite and positive')
        if not math.isfinite(self.p['max_speed']) or self.p['max_speed'] < 0:
            raise ValueError('max_speed must be nonnegative; 0 disables target slew limiting')
        for side in self.sides:
            for kind in ('abduction_gains', 'abduction_offsets'):
                v = self.p[f'{side}_{kind}']
                if len(v) != 5 or not all(math.isfinite(x) for x in v):
                    raise ValueError(f'{side}_{kind} must contain 5 finite numbers')
            for kind in ('gains', 'offsets', 'lower', 'upper'):
                v = self.p[f'{side}_{kind}']
                if len(v) != 15 or not all(math.isfinite(x) for x in v):
                    raise ValueError(f'{side}_{kind} must contain 15 finite numbers')
            for lo, hi, (_, _, hard_max) in zip(
                    self.p[f'{side}_lower'], self.p[f'{side}_upper'], FLEXION):
                if not 0.0 <= lo <= hi <= hard_max:
                    raise ValueError(f'{side} limits require 0 <= lower <= upper <= hard limit')
            if not self.p['dry_run'] and not self.p[f'{side}_glove_id']:
                raise ValueError(f'{side}_glove_id is required for hardware commands')
        self.lock = threading.Lock()
        self.frames = {}
        self.logged_gloves = set()
        self.feedback = {}
        self.command = {}
        self.enabled = False
        self.auto_arm_pending = self.p['auto_enable']
        self.source_present = False
        self.client = None
        self.last_tick = time.monotonic()
        self.publishers_ = {
            side: self.create_publisher(JointTrajectory,
                f'/contactglove/{side}/preview' if self.p['dry_run'] else
                f'/hand_{"l" if side == "left" else "r"}_controller/joint_trajectory', 1)
            for side in self.sides}
        self.create_subscription(JointState, '/joint_states', self.on_joints,
                                 qos_profile_sensor_data)
        self.create_service(SetBool, '~/enable', self.on_enable)
        self.create_timer(1.0 / self.p['rate'], self.tick)
        if self.p['input_mode'] == 'ros':
            from contact_glove_msgs.msg import Ergonomics
            self.source_topics = [self.p['glove_namespace'].rstrip('/') + f'/{s}/ergonomics'
                                  for s in self.sides]
            for side, topic in zip(self.sides, self.source_topics):
                self.create_subscription(
                    Ergonomics, topic,
                    lambda msg, expected=side: self.on_ros_frame(msg, expected),
                    qos_profile_sensor_data)
            self.get_logger().info(
                f'Waiting for {self.source_topics}; auto_enable={self.p["auto_enable"]}, '
                f'dry_run={self.p["dry_run"]}')
            return
        if self.p['input_mode'] != 'sdk':
            raise ValueError('input_mode must be sdk or ros')
        from contactglovesdk import ContactGloveClient, Host
        self.host = Host('DivingStation', self.p['host'], self.p['port'])
        self.client = ContactGloveClient(local_port=self.p['local_port'])
        self.client.register_callback_for_ergonomics_stream(self.on_frame)
        try:
            self.client.connect_to_host(self.host)
        except Exception:
            self.client.close()
            raise
        self.get_logger().info(
            f'DivingStation requested; receive port {self.client.local_port}; '
            f'dry_run={self.p["dry_run"]}. Enable with ~/enable after checking data.')

    def names(self, side):
        return [f'{side}_{self.mapping[i][0]}' for i in self.indices]

    def on_ros_frame(self, msg, expected):
        if ('left' if msg.is_left else 'right') == expected:
            self.on_frame(ergonomics_frame(msg))

    def on_joints(self, msg):
        now = time.monotonic()
        self.feedback.update({n: (float(v), now) for n, v in zip(msg.name, msg.position)
                              if math.isfinite(v)})

    def on_frame(self, frame):
        side = 'left' if frame.is_left else 'right'
        if side not in self.sides:
            return
        wanted = self.p[f'{side}_glove_id']
        if wanted and frame.id != wanted:
            return
        with self.lock:
            old = self.frames.get(side)
            # NotAttempted is an intentional IK skip in DivingStation. Do not use
            # that pose or refresh freshness; the last valid frame still expires.
            if int(frame.tracking_state) == 2:
                return
            # Repeated/reordered source frames must not refresh the watchdog.
            if old and frame.id == old[1] and frame.timestamp_ms <= old[2]:
                return
            try:
                if int(frame.tracking_state) != 0:
                    raise ValueError('tracking is not Tracked')
                if abs(time.time() - frame.timestamp_ms / 1000) > self.p['max_frame_age']:
                    raise ValueError('frame timestamp is stale; synchronize host clocks')
                goal = targets(frame.values, self.p[f'{side}_gains'],
                               self.p[f'{side}_offsets'], self.p[f'{side}_lower'],
                               self.p[f'{side}_upper'], clamp=self.p['clamp_targets'])
                if self.p['include_abduction']:
                    goal += abduction_targets(frame.values, self.p[f'{side}_abduction_gains'],
                                              self.p[f'{side}_abduction_offsets'],
                                              clamp=self.p['clamp_targets'])
                goal = [goal[i] for i in self.indices]
            except (ValueError, TypeError):
                self.frames.pop(side, None)
                return
            first = (side, frame.id) not in self.logged_gloves
            self.logged_gloves.add((side, frame.id))
            self.frames[side] = (time.monotonic(), frame.id, frame.timestamp_ms, goal)
        if first:
            self.get_logger().info(f'{side} glove id={frame.id}: valid ergonomics received')

    def ready(self, now):
        for side in self.sides:
            if (not self.p['dry_run'] and
                    self.publishers_[side].get_subscription_count() == 0):
                return False
            if side not in self.frames or now - self.frames[side][0] > self.p['timeout']:
                return False
            for name in self.names(side):
                if name not in self.feedback or now - self.feedback[name][1] > self.p['timeout']:
                    return False
        return True

    def publish(self, side, values):
        msg = JointTrajectory()
        msg.joint_names = self.names(side)
        point = JointTrajectoryPoint()
        point.positions = list(values)
        ns = int(self.p['horizon'] * 1e9)
        point.time_from_start = Duration(sec=ns // 1000000000, nanosec=ns % 1000000000)
        msg.points = [point]
        self.publishers_[side].publish(msg)

    def stop(self):
        if self.enabled and rclpy.ok():
            now = time.monotonic()
            for side, values in self.command.items():
                names = self.names(side)
                if all(n in self.feedback and now-self.feedback[n][1] <= self.p['timeout']
                       for n in names):
                    values = [self.feedback[n][0] for n in names]
                self.publish(side, values)
        self.enabled = False

    def on_enable(self, request, response):
        with self.lock:
            if not request.data:
                self.stop()
                self.auto_arm_pending = False
                response.success, response.message = True, 'Disabled; holds last target'
            elif not self.ready(time.monotonic()):
                response.success, response.message = False, 'Need fresh tracked glove and joint feedback'
            else:
                self.command = {s: [self.feedback[n][0] for n in self.names(s)] for s in self.sides}
                self.last_tick = time.monotonic()
                self.enabled = True
                self.auto_arm_pending = False
                response.success, response.message = True, 'Enabled from measured pose'
        return response

    def tick(self):
        now = time.monotonic()
        with self.lock:
            dt = min(now - self.last_tick, 1.0 / self.p['rate'])
            self.last_tick = now
            if self.p['input_mode'] == 'ros':
                present = all(self.count_publishers(t) > 0 for t in self.source_topics)
                if self.source_present and not present:
                    self.stop()
                    self.frames.clear()
                    self.auto_arm_pending = self.p['auto_enable']
                    self.get_logger().info('Glove bridge stopped; holding. Waiting for relaunch.')
                self.source_present = present
            if (not self.enabled and self.auto_arm_pending and self.ready(now) and
                    (self.p['input_mode'] != 'ros' or self.source_present)):
                self.command = {s: [self.feedback[n][0] for n in self.names(s)] for s in self.sides}
                self.enabled = True
                self.auto_arm_pending = False
                self.get_logger().info('Glove bridge ready: following enabled')
            if not self.enabled:
                return
            if not self.ready(now):
                self.stop()
                self.get_logger().error('Input/feedback lost: disabled. Explicitly enable to resume.')
                return
            for side in self.sides:
                self.command[side] = (slew(self.command[side], self.frames[side][3],
                                          self.p['max_speed'] * dt)
                                      if self.p['max_speed'] > 0 else list(self.frames[side][3]))
                self.publish(side, self.command[side])

    def close(self):
        self.stop()
        flush_holds(self.publishers_.values())
        if self.client is not None:
            self.client.disconnect(self.host)
            self.client.close()


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = None
    try:
        node = ContactGloveSharpa()
        spin_until_stopped(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
