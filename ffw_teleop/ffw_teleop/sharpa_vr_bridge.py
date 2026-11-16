"""Connect official ROBOTIS Vuer/Cyclo arms to SG2 without commanding Sharpa fingers."""
import copy
import math
import time
from pathlib import Path

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from rclpy.parameter import Parameter
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float32, String
from std_srvs.srv import SetBool, Trigger
import yaml
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from .vr_pose_math import bounded_goal, mount_matrix, pose_matrix, relative_goal, headset_heading_axes
from .stop_signals import flush_holds, spin_until_stopped
from .teleop_recording import SessionRecorder

SIDES = ('left', 'right')


class SharpaVRBridge(Node):
    def __init__(self):
        super().__init__('sharpa_vr_bridge')
        defaults = dict(dry_run=True, require_squeeze=True, squeeze_threshold=0.8,
                        timeout=0.3, feedback_timeout=0.5, rate=50.0,
                        relative_pose=True, position_scale=1.0, trajectory_time=0.05,
                        max_pose_step_m=0.25, max_pose_step_deg=60.0,
                        goal_speed_mps=0.8, goal_speed_radps=3.0,
                        ik_timeout=1.0, startup_grace=4.0,
                        calibration_file='',
                        motion_calibration_file='', world_reference_required=False,
                        preserve_orientation_reference=False,
                        use_headset_heading=False, position_y_sign=1.0,
                        left_axes_rpy_deg=[0.0]*3, right_axes_rpy_deg=[0.0]*3,
                        left_position_scale=1.0, right_position_scale=1.0,
                        left_mount_xyz=[0.0]*3, right_mount_xyz=[0.0]*3,
                        left_mount_rpy_deg=[0.0]*3, right_mount_rpy_deg=[0.0]*3)
        self.p = {k: self.declare_parameter(k, v).value for k, v in defaults.items()}
        calibration_path = Path(self.p['calibration_file']).expanduser() if self.p['calibration_file'] else None
        if calibration_path is not None and calibration_path.is_file():
            saved = yaml.safe_load(calibration_path.read_text())['sharpa_vr_bridge']['ros__parameters']
            for side in SIDES:
                for suffix in ('mount_xyz', 'mount_rpy_deg'):
                    key = f'{side}_{suffix}'
                    if key in saved:
                        self.p[key] = [float(x) for x in saved[key]]
                        self.set_parameters([Parameter(key, value=self.p[key])])
        motion_file = self.p['motion_calibration_file']
        if motion_file and Path(motion_file).expanduser().is_file():
            saved = yaml.safe_load(Path(motion_file).expanduser().read_text())['sharpa_vr_bridge']['ros__parameters']
            if saved.get('motion_reference') != 'world_v1':
                raise ValueError('Motion calibration must use world_v1 coordinates')
            self.p['world_reference_required'] = True
            for side in SIDES:
                for suffix in ('axes_rpy_deg', 'position_scale', 'mount_xyz'):
                    key = f'{side}_{suffix}'
                    self.p[key] = saved[key]
                    self.set_parameters([Parameter(key, value=self.p[key])])
        self.axes = {s: mount_matrix([0.0]*3, self.p[f'{s}_axes_rpy_deg'])[:3, :3] for s in SIDES}
        for key in ('timeout', 'feedback_timeout', 'rate', 'position_scale',
                    'left_position_scale', 'right_position_scale',
                    'trajectory_time', 'max_pose_step_m', 'max_pose_step_deg',
                    'goal_speed_mps', 'goal_speed_radps', 'ik_timeout', 'startup_grace'):
            if not math.isfinite(self.p[key]) or self.p[key] <= 0:
                raise ValueError(f'{key} must be finite and positive')
        if not 0.0 <= self.p['squeeze_threshold'] <= 1.0:
            raise ValueError('squeeze_threshold must be in [0, 1]')
        if self.p['position_y_sign'] not in (-1., 1.):
            raise ValueError('position_y_sign must be +1 or -1')
        self.mounts = {s: mount_matrix(self.p[f'{s}_mount_xyz'], self.p[f'{s}_mount_rpy_deg'])
                       for s in SIDES}
        self.poses, self.grips, self.feedback, self.anchors = {}, {}, {}, {}
        self.orientation_references = {}
        self.headset_pose = None
        self.heading_registered = False
        self.recording = SessionRecorder('vr-bridge')
        self.recording.record('configuration', self.p)
        self.active = False
        self.filtered_goals = {}
        self.last_ik = {}
        self.armed_at = 0.0
        self.last_tick = time.monotonic()
        self.last_stop_reason = ''
        self.wait_reason = ''
        self.world_reference_seen = -math.inf
        self.create_subscription(String, '/vr/input/reference_frame', self.on_reference_frame, 10)
        self.create_subscription(PoseStamped, '/vr/input/headset_pose', self.on_headset_pose, 10)
        self.model_state_pub = self.create_publisher(JointState, '/vr/model/joint_states', 10)
        self.goal_pubs = {}
        self.arm_pubs = {}
        for side, short in (('left', 'l'), ('right', 'r')):
            self.goal_pubs[side] = self.create_publisher(PoseStamped, f'/vr/{side}/goal_pose', 10)
            output = f'/vr/preview/{side}/joint_trajectory' if self.p['dry_run'] else \
                f'/arm_{short}_controller/joint_trajectory'
            self.arm_pubs[side] = self.create_publisher(JointTrajectory, output, 1)
            self.create_subscription(PoseStamped, f'/vr/raw/{side}/wrist_pose',
                                     lambda m, s=side: self.on_pose(m, s, 'vr'), qos_profile_sensor_data)
            self.create_subscription(PoseStamped, f'/vr/state/{side}/wrist_pose',
                                     lambda m, s=side: self.on_pose(m, s, 'robot'), qos_profile_sensor_data)
            self.create_subscription(Float32, f'/vr/input/{side}/squeeze',
                                     lambda m, s=side: self.on_grip(m, s), qos_profile_sensor_data)
            self.create_subscription(JointTrajectory, f'/vr/ik/{side}/joint_trajectory',
                                     lambda m, s=side: self.on_trajectory(m, s), 10)
        self.control_pub = self.create_publisher(Bool, '/vr/control/reactivate', 10)
        self.active_pub = self.create_publisher(Bool, '/vr/active', 10)
        self.create_subscription(Bool, '/vr/input/activate', self.on_request, 10)
        self.create_subscription(Bool, '/vr/reference_diverged', self.on_divergence, 10)
        self.create_subscription(JointState, '/joint_states', self.on_joints, qos_profile_sensor_data)
        self.create_service(SetBool, '~/enable', self.on_enable)
        self.create_service(Trigger, '~/calibrate_mount', self.on_calibrate_mount)
        self.create_timer(1.0 / self.p['rate'], self.tick)
        self.get_logger().info(f'VR arms ready; dry_run={self.p["dry_run"]}; awaiting explicit activation')
        self.get_logger().info(f'Telemetry: {self.recording.path}')

    @staticmethod
    def names(side):
        return [f'arm_{"l" if side == "left" else "r"}_joint{i}' for i in range(1, 8)]

    def destroy_node(self):
        if hasattr(self, 'recording'):
            self.recording.close()
        return super().destroy_node()

    def on_reference_frame(self, msg):
        if msg.data == 'world_v1':
            self.world_reference_seen = time.monotonic()
        else:
            self.world_reference_seen = -math.inf
            if self.p['world_reference_required']:
                self.stop('VR reference frame changed; recalibrate')

    def on_headset_pose(self, msg):
        self.recording.record('headset_pose', msg, every=.05)
        try:
            stamp = msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
            if msg.header.frame_id != 'base_link' or abs(self.get_clock().now().nanoseconds/1e9-stamp) > 2.:
                return
            self.headset_pose = (pose_matrix(msg.pose), time.monotonic())
        except ValueError:
            self.headset_pose = None

    def on_joints(self, msg):
        self.recording.record('measured_joints', msg, every=.05)
        if len(msg.name) != len(msg.position) or not all(math.isfinite(v) for v in msg.position):
            self.feedback.clear()
            self.stop('Invalid robot joint feedback')
            return
        now = time.monotonic()
        for name, value in zip(msg.name, msg.position):
            if math.isfinite(value):
                self.feedback[name] = (float(value), now)
            else:
                self.feedback.pop(name, None)
        state = copy.deepcopy(msg)
        state.velocity = [float(v) if math.isfinite(v) else 0.0 for v in msg.velocity]
        self.model_state_pub.publish(state)

    def on_grip(self, msg, side):
        if math.isfinite(msg.data) and 0.0 <= msg.data <= 1.0:
            self.grips[side] = (float(msg.data), time.monotonic())
        else:
            self.grips.pop(side, None)
        if self.active and self.p['require_squeeze'] and not self.grip_ok(side, time.monotonic()):
            self.stop('Grip released or invalid')

    def on_pose(self, msg, side, kind):
        self.recording.record(f'{kind}_pose_{side}', msg, every=.05)
        key = (kind, side)
        try:
            if msg.header.frame_id != 'base_link':
                raise ValueError('Pose frame must be base_link')
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
            if abs(self.get_clock().now().nanoseconds / 1e9 - stamp) > self.p['feedback_timeout']:
                raise ValueError('Stale pose timestamp; synchronize clocks')
            previous = self.poses.get(key)
            if previous and stamp <= previous[2]:
                return
            matrix = pose_matrix(msg.pose)
            if kind == 'vr':
                matrix = matrix @ self.mounts[side]
                if previous and self.active:
                    distance = np.linalg.norm(matrix[:3, 3] - previous[0][:3, 3])
                    angle = Rotation.from_matrix(matrix[:3, :3] @ previous[0][:3, :3].T).magnitude()
                    if distance > self.p['max_pose_step_m'] or math.degrees(angle) > self.p['max_pose_step_deg']:
                        raise ValueError(
                            f'VR tracking pose jumped: {side}, '
                            f'distance={distance:.3f} m, rotation={math.degrees(angle):.1f} deg, '
                            f'interval={stamp - previous[2]:.3f} s '
                            f'(limits {self.p["max_pose_step_m"]:.3f} m / '
                            f'{self.p["max_pose_step_deg"]:.1f} deg)')
            self.poses[key] = (matrix, time.monotonic(), stamp)
        except ValueError as error:
            self.poses.pop(key, None)
            self.stop(str(error))

    def grip_ok(self, side, now):
        value, stamp = self.grips.get(side, (0.0, -math.inf))
        return now - stamp <= self.p['timeout'] and value >= self.p['squeeze_threshold']

    def ready(self, now):
        needs_heading = self.p['use_headset_heading'] and (not self.heading_registered
            or (not self.active and not self.p['preserve_orientation_reference']))
        if needs_heading and (self.headset_pose is None or now-self.headset_pose[1] > 2.):
            self.wait_reason = 'Quest heading missing/stale; Enter VR with the headset tracking'
            return False
        if self.p['world_reference_required'] and now-self.world_reference_seen > 2.5:
            self.wait_reason = 'World-frame VR publisher missing/stale'
            return False
        for side in SIDES:
            for kind in ('vr', 'robot'):
                sample = self.poses.get((kind, side))
                timeout = self.p['timeout'] if kind == 'vr' else self.p['feedback_timeout']
                if sample is None or now - sample[1] > timeout:
                    age = 'missing' if sample is None else f'{now-sample[1]:.3f}s old'
                    self.wait_reason = f'{side} {kind} wrist pose: {age}'
                    return False
            for name in self.names(side):
                if name not in self.feedback or now - self.feedback[name][1] > self.p['feedback_timeout']:
                    self.wait_reason = f'Joint feedback missing/stale: {name}'
                    return False
            if self.p['require_squeeze'] and not self.grip_ok(side, now):
                self.wait_reason = f'{side} grip missing/released'
                return False
            if not self.p['dry_run'] and self.arm_pubs[side].get_subscription_count() == 0:
                self.wait_reason = f'{side} arm controller has no trajectory subscriber'
                return False
        self.wait_reason = ''
        return True

    def enable(self):
        if self.active:
            return True, 'Already following'
        if not self.ready(time.monotonic()):
            return False, self.wait_reason
        if self.p['use_headset_heading'] and (not self.heading_registered
                or not self.p['preserve_orientation_reference']):
            try:
                axes = headset_heading_axes(self.headset_pose[0][:3, :3])
            except ValueError as error:
                return False, str(error)
            self.axes = {s: axes.copy() for s in SIDES}
            self.heading_registered = True
        self.anchors = {s: (self.poses[('vr', s)][0].copy(), self.poses[('robot', s)][0].copy())
                        for s in SIDES}
        if self.p['preserve_orientation_reference']:
            for side in SIDES:
                vr, robot = self.anchors[side]
                if side not in self.orientation_references:
                    self.orientation_references[side] = (vr[:3, :3].copy(), robot[:3, :3].copy())
                vr[:3, :3], robot[:3, :3] = self.orientation_references[side]
        self.recording.record('anchor', {s: dict(vr=self.anchors[s][0], robot=self.anchors[s][1],
            axes=self.axes[s], mount=self.mounts[s]) for s in SIDES})
        self.active = True
        self.armed_at = time.monotonic()
        self.last_tick = self.armed_at
        self.last_ik = {s: self.armed_at + self.p['startup_grace'] for s in SIDES}
        self.filtered_goals = {s: self.anchors[s][1].copy() for s in SIDES}
        self.last_stop_reason = ''
        self.publish_goals(0.0)
        self.control_pub.publish(Bool(data=True))
        self.get_logger().info('VR armed from measured wrist poses; Cyclo startup alignment remains active')
        return True, 'Enabled from measured wrist poses'

    def on_request(self, msg):
        if msg.data:
            ok, reason = self.enable()
            if not ok:
                self.get_logger().warning(reason)
        else:
            self.stop('VR controller requested stop')

    def on_divergence(self, msg):
        # Cyclo itself ignores reference-checker jumps during its 3-second
        # activation delay, including the anchor reset between sessions.
        if msg.data and self.active and time.monotonic()-self.armed_at > 3.2:
            self.stop('Cyclo reference divergence')

    def on_enable(self, request, response):
        if request.data:
            response.success, response.message = self.enable()
        else:
            self.stop('Stopped by service')
            response.success, response.message = True, 'Stopped; arm targets replaced by measured hold'
        return response

    def on_calibrate_mount(self, request, response):
        if not self.ready(time.monotonic()):
            response.success, response.message = False, self.wait_reason
            return response
        self.stop('Recalibrating glove controller mounts')
        mounts, saved = {}, {}
        for side in SIDES:
            raw = self.poses[('vr', side)][0] @ np.linalg.inv(self.mounts[side])
            robot = self.poses[('robot', side)][0]
            correction = raw[:3, :3].T @ robot[:3, :3]
            rpy = Rotation.from_matrix(correction).as_euler('xyz', degrees=True).tolist()
            saved[f'{side}_mount_rpy_deg'] = rpy
            saved[f'{side}_mount_xyz'] = list(self.p[f'{side}_mount_xyz'])
            mounts[side] = mount_matrix(saved[f'{side}_mount_xyz'], rpy)
        if self.p['calibration_file']:
            try:
                path = Path(self.p['calibration_file']).expanduser()
                path.parent.mkdir(parents=True, exist_ok=True)
                temporary = path.with_suffix(path.suffix+'.tmp')
                temporary.write_text(yaml.safe_dump({'sharpa_vr_bridge': {'ros__parameters': saved}}))
                temporary.replace(path)
            except OSError as error:
                response.success, response.message = False, f'Calibration not applied: {error}'
                return response
        for side in SIDES:
            old = self.poses[('vr', side)]
            raw = old[0] @ np.linalg.inv(self.mounts[side])
            self.poses[('vr', side)] = (raw @ mounts[side], old[1], old[2])
        self.mounts = mounts
        for key, value in saved.items():
            self.p[key] = value
            self.set_parameters([Parameter(key, value=value)])
        self.anchors.clear()
        self.orientation_references.clear()
        self.filtered_goals.clear()
        response.success = True
        response.message = 'Mount orientations calibrated to measured wrists; following remains stopped'
        self.get_logger().info(response.message)
        return response

    def stop(self, reason):
        if reason != self.last_stop_reason:
            self.recording.record('stop', {'reason': reason, 'was_active': self.active})
        was_active = self.active
        self.active = False
        self.control_pub.publish(Bool(data=False))
        self.active_pub.publish(Bool(data=False))
        if was_active:
            now = time.monotonic()
            for side in SIDES:
                names = self.names(side)
                if all(n in self.feedback and now-self.feedback[n][1] <= self.p['feedback_timeout']
                       for n in names):
                    self.publish_arm(side, [self.feedback[n][0] for n in names])
            self.get_logger().warning(reason + '; VR arms stopped, explicit reactivation required')
        self.last_stop_reason = reason

    def publish_arm(self, side, positions):
        self.recording.record(f'arm_command_{side}', positions, every=.05)
        ns = int(self.p['trajectory_time'] * 1e9)
        point = JointTrajectoryPoint(positions=list(positions),
            time_from_start=Duration(sec=ns//1000000000, nanosec=ns%1000000000))
        self.arm_pubs[side].publish(JointTrajectory(joint_names=self.names(side), points=[point]))

    def on_trajectory(self, msg, side):
        if not self.active:
            return
        if not self.ready(time.monotonic()):
            self.stop('Input/feedback lost: ' + self.wait_reason)
            return
        expected = self.names(side)
        if (len(msg.points) != 1 or len(msg.joint_names) != 7 or set(msg.joint_names) != set(expected)
                or len(msg.points[0].positions) != 7
                or not all(math.isfinite(v) for v in msg.points[0].positions)):
            self.stop('Cyclo output is not exactly seven finite arm joint positions')
            return
        positions = dict(zip(msg.joint_names, msg.points[0].positions))
        self.last_ik[side] = time.monotonic()
        self.publish_arm(side, [positions[n] for n in expected])

    def publish_goals(self, dt):
        for side in SIDES:
            current = self.poses[('vr', side)][0]
            matrix = relative_goal(current, *self.anchors[side],
                self.p['position_scale']*self.p[f'{side}_position_scale'], self.axes[side],
                self.p['position_y_sign']) \
                if self.p['relative_pose'] else current
            if (self.p['preserve_orientation_reference']
                    and time.monotonic()-self.armed_at < 3.5):
                matrix = self.filtered_goals[side]
            matrix = bounded_goal(self.filtered_goals[side], matrix,
                                  self.p['goal_speed_mps']*dt, self.p['goal_speed_radps']*dt)
            self.filtered_goals[side] = matrix
            msg = PoseStamped()
            msg.header.frame_id = 'base_link'
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = matrix[:3, 3].tolist()
            q = Rotation.from_matrix(matrix[:3, :3]).as_quat().tolist()
            (msg.pose.orientation.x, msg.pose.orientation.y,
             msg.pose.orientation.z, msg.pose.orientation.w) = q
            self.goal_pubs[side].publish(msg)
            self.recording.record(f'goal_pose_{side}', msg, every=.05)

    def tick(self):
        now = time.monotonic()
        dt = min(now-self.last_tick, 1.0/self.p['rate'])
        self.last_tick = now
        if self.active:
            if not self.ready(now):
                self.stop('Input/feedback lost: ' + self.wait_reason)
            elif any(now-self.last_ik[s] > self.p['ik_timeout'] for s in SIDES):
                self.stop('Cyclo arm output stopped')
            else:
                self.publish_goals(dt)
        self.control_pub.publish(Bool(data=self.active))
        self.active_pub.publish(Bool(data=self.active))


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = None
    try:
        node = SharpaVRBridge()
        spin_until_stopped(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            if rclpy.ok():
                node.stop('Bridge shutdown')
                flush_holds([*node.arm_pubs.values(), node.control_pub])
            node.destroy_node()
            node.recording.close()
        if rclpy.ok():
            rclpy.shutdown()
