"""Official SG2 Vuer with a same-origin WebSocket URL for a bare browser URL."""
from .teleop_recording import SessionRecorder


class WorldReferenceMixin:
    async def on_camera_move(self, event, session):
        import numpy as np
        import time
        from scipy.spatial.transform import Rotation
        from geometry_msgs.msg import PoseStamped
        from robotis_vuer.vr_publisher_sg2 import VR_HEAD_TO_ROS
        from .vr_pose_math import camera_to_head_bone
        camera = event.value.get('camera') if isinstance(event.value, dict) else None
        if not isinstance(camera, dict):
            return
        self.recording.record('camera_input', camera, every=.05)
        values = np.asarray(camera.get('matrix', []), dtype=float)
        if values.size != 16 or not np.isfinite(values).all():
            return
        matrix = values.reshape(4, 4, order='F')
        if abs(np.linalg.det(matrix[:3, :3])-1.) > .1:
            return
        msg = PoseStamped()
        msg.header.frame_id = 'base_link'
        msg.header.stamp = self.get_clock().now().to_msg()
        rotation = VR_HEAD_TO_ROS @ camera_to_head_bone(matrix[:3, :3])
        q = Rotation.from_matrix(rotation).as_quat().tolist()
        msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w = q
        self.headset_pose_pub.publish(msg)
        self.camera_seen = time.monotonic()
        self.recording.record('headset_pose', msg, every=.05)

    async def on_body_tracking_move(self, event, session):
        import numpy as np
        import time
        from scipy.spatial.transform import Rotation
        from geometry_msgs.msg import PoseStamped
        from robotis_vuer.vr_publisher_sg2 import BODY_HEAD_INDEX, VR_HEAD_TO_ROS
        if isinstance(event.value, dict):
            body = event.value.get('body')
            if isinstance(body, (list, tuple, np.ndarray)):
                values = np.asarray(body, dtype=float).reshape(-1)
                start = BODY_HEAD_INDEX*16
                if values.size >= start+16:
                    head = values[start:start+16].reshape(4, 4, order='F')
                    if (self.headset_pose_pub is not None and time.monotonic()-self.camera_seen > 1.
                            and np.isfinite(head).all()
                            and abs(np.linalg.det(head[:3, :3])-1.) < .1):
                        msg = PoseStamped()
                        msg.header.frame_id = 'base_link'
                        msg.header.stamp = self.get_clock().now().to_msg()
                        q = Rotation.from_matrix(VR_HEAD_TO_ROS @ head[:3, :3]).as_quat()
                        (msg.pose.orientation.x, msg.pose.orientation.y,
                         msg.pose.orientation.z, msg.pose.orientation.w) = q.tolist()
                        self.headset_pose_pub.publish(msg)
                        self.recording.record('headset_pose', msg, every=.05)
        return await super().on_body_tracking_move(event, session)

    async def on_controller_move(self, event, session):
        if isinstance(event.value, dict):
            data = event.value
            self.recording.record('controller_input', {
                'left': data.get('left'), 'right': data.get('right'),
                'leftState': data.get('leftState'), 'rightState': data.get('rightState')}, every=.05)
        return await super().on_controller_move(event, session)

    def _publish_synced_pose_frame_if_ready(self):
        import numpy as np
        # Never subtract the moving neck-mounted headset transform.
        self.head_inverse_matrix = np.eye(4)
        # World-space controller poses do not require BODY_MOVE frames. Neck
        # mounting can stop body tracking while controller tracking stays live.
        self.pending_body_pose_frame = True
        return super()._publish_synced_pose_frame_if_ready()


def main(args=None):
    import math
    from aiohttp import web
    from rcl_interfaces.msg import SetParametersResult
    import rclpy
    from robotis_vuer import vr_publisher_sg2 as official
    from vuer import Vuer

    class SameOriginVuer(Vuer):
        async def socket_index(self, request):
            websocket = request.headers.get('Upgrade', '').lower() == 'websocket'
            if not websocket and not request.query.get('ws'):
                query = dict(request.query)
                query['ws'] = f'{"wss" if request.secure else "ws"}://{request.host}'
                raise web.HTTPFound(location=str(request.rel_url.with_query(query)))
            return await super().socket_index(request)

    # Keep ROBOTIS controller events, deadman buttons and coordinate conversion.
    official.Vuer = SameOriginVuer
    rclpy.init(args=args)
    class WorldTrajectoryPublisher(WorldReferenceMixin, official.VRTrajectoryPublisher):
        def __init__(self):
            import math
            self.recording = SessionRecorder('vuer-input')
            from geometry_msgs.msg import PoseStamped
            self.headset_pose_pub = None
            self.camera_seen = -math.inf
            super().__init__()
            self.headset_pose_pub = self.create_publisher(PoseStamped, '/vr/input/headset_pose', 10)
            self.vuer.add_handler('CAMERA_MOVE')(self.on_camera_move)

    node = WorldTrajectoryPublisher()
    node.get_logger().info(f'Controller telemetry: {node.recording.path}')
    from std_msgs.msg import String
    from rclpy.qos import QoSProfile, DurabilityPolicy
    frame_pub = node.create_publisher(String, '/vr/input/reference_frame',
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    node.create_timer(1.0, lambda: frame_pub.publish(String(data='world_v1')))
    # The upstream 12 cm correction assumes a controller held in the palm.
    # A controller attached to the glove uses its tracked point, then the
    # explicitly measured mount_xyz in the arm bridge.
    node.controller_back_offset_m = float(node.declare_parameter('controller_back_offset_m', 0.0).value)
    def update_parameters(parameters):
        for parameter in parameters:
            if parameter.name == 'goal_pose_squeeze_threshold':
                value = parameter.value
                if not isinstance(value, (int, float)) or not 0.0 <= value <= 1.0:
                    return SetParametersResult(successful=False, reason='Threshold must be in [0, 1]')
            if parameter.name == 'controller_back_offset_m':
                if not math.isfinite(parameter.value) or not 0.0 <= parameter.value <= 0.3:
                    return SetParametersResult(successful=False, reason='Back offset must be in [0, 0.3] metres')
        for parameter in parameters:
            if parameter.name == 'goal_pose_squeeze_threshold':
                node.goal_pose_squeeze_threshold = float(parameter.value)
            if parameter.name == 'controller_back_offset_m':
                node.controller_back_offset_m = float(parameter.value)
        return SetParametersResult(successful=True)
    node.add_on_set_parameters_callback(update_parameters)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        node.recording.close()
        if rclpy.ok():
            rclpy.shutdown()
