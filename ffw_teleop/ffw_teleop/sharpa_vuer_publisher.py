"""Official SG2 Vuer with a same-origin WebSocket URL for a bare browser URL."""


class WorldReferenceMixin:
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
        pass

    node = WorldTrajectoryPublisher()
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
        if rclpy.ok():
            rclpy.shutdown()
