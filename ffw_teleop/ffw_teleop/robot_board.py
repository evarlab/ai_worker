"""Observe the printed board with the head camera. This module never commands motors."""
from collections import deque
import argparse
import json
from pathlib import Path
import time
import numpy as np
from .human_demo_data import rigid
from .human_demo_calibration import mean_transform, residuals
from .human_demo_robot import RobotFrames
from .human_demo_aruco import ArucoDetector


def rectified_calibration(width, height, frame, info):
    if width != info.width or height != info.height or frame != info.header.frame_id:
        raise ValueError('Image and CameraInfo dimensions/frame differ')
    p = np.asarray(info.p, dtype=float).reshape(3, 4)
    if not np.isfinite(p).all() or p[0, 0] <= 0 or p[1, 1] <= 0 or not np.allclose(p[:, 3], 0):
        raise ValueError('Expected calibrated rectified left-camera projection')
    return dict(K=p[:, :3].tolist(), distortion=[0.] * 5, source='ROS rectified left-camera CameraInfo.P')


def stable_board(transforms, expected_height=.7005, min_samples=20):
    if len(transforms) < min_samples:
        raise ValueError('Waiting for stable board samples')
    centre = mean_transform(transforms)
    distances, angles = residuals(transforms, centre)
    if max(distances) > .01 or max(angles) > np.deg2rad(2):
        raise ValueError('Board position/orientation is unstable')
    if centre[2, 2] < np.cos(np.deg2rad(10)):
        raise ValueError('Board is not face-up and horizontal')
    if abs(centre[2, 3] - expected_height) > .08:
        raise ValueError('Board height disagrees with the 700 mm table; check camera extrinsics')
    return dict(T_base_board=rigid(centre).tolist(), samples=len(transforms),
                position_max_m=float(max(distances)), rotation_max_deg=float(np.rad2deg(max(angles))),
                reference_frame='base_link', camera_extrinsics='nominal live URDF, not independently measured',
                expected_board_height_m=expected_height)


def main():
    import rclpy
    from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
    from sensor_msgs.msg import Image, CameraInfo, JointState
    from std_msgs.msg import String
    from geometry_msgs.msg import PoseStamped
    from cv_bridge import CvBridge
    import cv2
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--workspace', type=Path, required=True)
    p.add_argument('--image', default='/zed/zed_node/rgb/image_rect_color')
    p.add_argument('--info', default='/zed/zed_node/rgb/camera_info')
    p.add_argument('--output', type=Path, default=Path('/tmp/human_demo_robot_board.json'))
    p.add_argument('--expected-height', type=float, default=.7005)
    args = p.parse_args()
    if not np.isfinite(args.expected_height):p.error('expected height must be finite')
    workspace = json.loads(args.workspace.read_text())
    rclpy.init(); node = rclpy.create_node('human_demo_head_board')
    detector = ArucoDetector(workspace['dictionary']); bridge = CvBridge()
    state = dict(info=None, frames=None, urdf=None, last_image=0., last_print=0., status='waiting for camera', last_stamp=None)
    history = deque(maxlen=300); samples = deque(maxlen=20)
    pub = node.create_publisher(PoseStamped, '/human_demo/board_pose', 1)
    def save(value):
        args.output.parent.mkdir(parents=True, exist_ok=True)
        value.update(observed_monotonic_s=time.monotonic(), robot_ros_time_ns=node.get_clock().now().nanoseconds,
                     image_topic=args.image, info_topic=args.info)
        temp = args.output.with_suffix('.tmp'); temp.write_text(json.dumps(value, ensure_ascii=False, indent=2)); temp.replace(args.output)
    def invalid(reason):
        samples.clear(); state['status'] = reason
        save(dict(valid=False, reason=reason))
    def description(msg):
        if msg.data != state['urdf']:
            state['frames'] = RobotFrames(msg.data); state['urdf'] = msg.data; samples.clear()
    def joints(msg):
        stamp = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        history.append((stamp, dict(zip(msg.name, msg.position))))
    def image(msg):
        now = time.monotonic()
        if now-state['last_image'] < .15:return
        state['last_image'] = now
        try:
            if state['frames'] is None or state['info'] is None or not history:
                raise ValueError('Waiting for URDF, CameraInfo and joint feedback')
            stamp = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
            if state['last_stamp'] is not None and stamp <= state['last_stamp']:
                raise ValueError('Repeated or reversed camera timestamp')
            state['last_stamp'] = stamp
            ros_age = (node.get_clock().now().nanoseconds-stamp)/1e9
            if not -.1 <= ros_age <= .5:raise ValueError('Camera image is stale')
            joint_stamp, values = min(history, key=lambda item: abs(item[0]-stamp))
            if abs(joint_stamp-stamp) > 100_000_000:raise ValueError('No joint feedback within 100 ms of image')
            calibration = rectified_calibration(msg.width, msg.height, msg.header.frame_id, state['info'])
            im = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            ok, jpg = cv2.imencode('.jpg', im)
            if not ok:raise ValueError('Image encoding failed')
            result = detector.detect(jpg.tobytes(), calibration, workspace)
            for marker in result['markers']:
                pts = np.int32(marker['corners']);cv2.polylines(im,[pts],True,(0,255,0),2)
                cv2.putText(im,str(marker['id']),tuple(pts[0]),cv2.FONT_HERSHEY_SIMPLEX,.7,(0,255,0),2)
            cv2.imwrite(str(args.output.with_suffix('.jpg')), im)
            pose = result['pose']
            if not pose or len(set(pose['marker_ids'])) < 3:raise ValueError('Need at least 3 known board markers')
            base_camera = state['frames'].transforms(values).get(msg.header.frame_id)
            if base_camera is None:raise ValueError('Camera optical frame is absent from live URDF')
            board = rigid(base_camera) @ rigid(pose['T_camera_board'])
            samples.append(board)
            try: fitted = stable_board(samples, args.expected_height)
            except ValueError as error:
                state['status'] = str(error) + ' (%d/20)' % len(samples)
                save(dict(valid=False, reason=state['status'], marker_ids=pose['marker_ids'], T_base_board_candidate=board.tolist(), image_stamp_ns=stamp, joint_stamp_ns=joint_stamp, reprojection_px=pose['reprojection_px']))
                return
            fitted.update(valid=True, image_stamp_ns=stamp, joint_stamp_ns=joint_stamp,
                          marker_ids=pose['marker_ids'], reprojection_px=pose['reprojection_px'],
                          camera_frame=msg.header.frame_id)
            save(fitted)
            from scipy.spatial.transform import Rotation
            centre = np.asarray(fitted['T_base_board']);q = Rotation.from_matrix(centre[:3,:3]).as_quat()
            out = PoseStamped();out.header.stamp = msg.header.stamp;out.header.frame_id = 'base_link'
            out.pose.position.x, out.pose.position.y, out.pose.position.z = map(float,centre[:3,3])
            out.pose.orientation.x,out.pose.orientation.y,out.pose.orientation.z,out.pose.orientation.w = map(float,q)
            pub.publish(out)
            state['status'] = 'READY IDs=%s board XYZ=%s m error=%.2f px' % (pose['marker_ids'],np.round(centre[:3,3],3).tolist(),pose['reprojection_px'])
        except (ValueError, KeyError, cv2.error) as error:
            invalid(str(error))
    node.create_subscription(String, '/robot_description', description, QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    node.create_subscription(JointState, '/joint_states', joints, qos_profile_sensor_data)
    node.create_subscription(CameraInfo, args.info, lambda msg: state.update(info=msg), qos_profile_sensor_data)
    node.create_subscription(Image, args.image, image, qos_profile_sensor_data)
    def status():
        if time.monotonic()-state['last_image'] > 1.:
            invalid('No fresh camera image')
        print(state['status'], flush=True)
    node.create_timer(1., status)
    save(dict(valid=False,reason='Observer starting'))
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        save(dict(valid=False,reason='Observer stopped'));node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__ == '__main__':main()
