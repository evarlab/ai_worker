import math
from pathlib import Path
import time
import xml.etree.ElementTree as ET

import numpy as np
import pytest

from ffw_teleop.sharpa_vr_model import prepare_model, simplify_collision_meshes
from ffw_teleop.vr_pose_math import bounded_goal, mount_matrix, pose_matrix, relative_goal


def model_fixture():
    robot = ET.Element('robot', name='test')
    for link in ('base_link', 'left_adapter_mount', 'right_adapter_mount', 'left_finger', 'right_finger', 'wheel'):
        ET.SubElement(robot, 'link', name=link)
    for side in ('l', 'r'):
        for i in range(1, 8):
            name = f'arm_{side}_link{i}'
            ET.SubElement(robot, 'link', name=name)
            joint = ET.SubElement(robot, 'joint', name=f'arm_{side}_joint{i}', type='revolute')
            ET.SubElement(joint, 'parent', link='base_link')
            ET.SubElement(joint, 'child', link=name)
            ET.SubElement(joint, 'limit', velocity='2', effort='10', lower='-2', upper='2')
    for side in ('left', 'right'):
        joint = ET.SubElement(robot, 'joint', name=side+'_finger_joint', type='revolute')
        ET.SubElement(joint, 'parent', link=side+'_adapter_mount')
        ET.SubElement(joint, 'child', link=side+'_finger')
        ET.SubElement(joint, 'limit', velocity='3', effort='1', lower='-1', upper='1')
    joint = ET.SubElement(robot, 'joint', name='wheel_joint', type='continuous')
    ET.SubElement(joint, 'parent', link='base_link')
    ET.SubElement(joint, 'child', link='wheel')
    ET.SubElement(joint, 'limit', velocity='20', effort='1')
    return robot


def test_model_locks_fingers_but_keeps_their_live_coordinates():
    xml, srdf = prepare_model(ET.tostring(model_fixture()), '', lambda _: '')
    joints = {j.get('name'): j for j in ET.fromstring(xml).findall('joint')}
    assert joints['arm_l_joint1'].find('limit').get('velocity') == '2'
    assert joints['left_finger_joint'].get('type') == 'revolute'
    assert joints['left_finger_joint'].find('limit').get('velocity') == '0'
    assert joints['wheel_joint'].get('type') == 'revolute'
    assert float(joints['wheel_joint'].find('limit').get('lower')) < -1000
    disabled = {frozenset((x.get('link1'), x.get('link2')))
                for x in ET.fromstring(srdf).findall('disable_collisions')}
    assert frozenset(('left_adapter_mount', 'left_finger')) in disabled
    assert frozenset(('left_finger', 'right_finger')) not in disabled
    assert frozenset(('left_finger', 'base_link')) not in disabled


def test_model_rejects_wrong_robot_and_missing_mesh(tmp_path):
    robot = model_fixture()
    robot.remove(robot.find("joint[@name='arm_l_joint7']"))
    with pytest.raises(ValueError, match='14 SG2'):
        prepare_model(ET.tostring(robot), '', lambda _: '')
    robot = model_fixture()
    geometry = ET.SubElement(ET.SubElement(robot.find('link'), 'collision'), 'geometry')
    ET.SubElement(geometry, 'mesh', filename='package://test/missing.stl')
    with pytest.raises(FileNotFoundError):
        prepare_model(ET.tostring(robot), '', lambda _: tmp_path)


def test_relative_anchor_and_controller_mount():
    initial = mount_matrix([0.5, 0.1, 1.0], [0., 0., 30.])
    robot = mount_matrix([0.4, 0.3, 0.8], [90., 0., 0.])
    assert np.allclose(relative_goal(initial, initial, robot, 1.0), robot)
    current = initial.copy()
    current[:3, 3] += [0.1, 0.0, 0.0]
    assert np.allclose(relative_goal(current, initial, robot, .5)[:3, 3], [.45, .3, .8])
    mounted = mount_matrix([0., 0., 0.], [0., 0., 90.]) @ mount_matrix([.1, 0., 0.], [0., 0., 0.])
    assert np.allclose(mounted[:3, 3], [0., .1, 0.], atol=1e-9)


def test_collision_boxes_enclose_scaled_mesh_in_rotated_frame(tmp_path):
    filename = tmp_path/'mesh.stl'
    filename.write_text('solid t\n facet normal 0 0 1\n outer loop\n'
                        'vertex 0 0 0\nvertex 2 0 0\nvertex 0 4 1\n'
                        'endloop\nendfacet\nendsolid t\n')
    robot = ET.fromstring(f'<robot><link name="l"><collision>'
                          '<origin xyz="1 0 0" rpy="0 0 1.5707963267948966"/>'
                          f'<geometry><mesh filename="{filename}" scale="2 1 1"/>'
                          '</geometry></collision></link></robot>')
    simplify_collision_meshes(robot)
    collision = robot.find('./link/collision')
    assert list(map(float, collision.find('geometry/box').get('size').split())) == [4., 4., 1.]
    assert list(map(float, collision.find('origin').get('xyz').split())) == pytest.approx([-1., 2., .5])


def test_model_resolves_file_uri_before_building_collision_box(tmp_path):
    filename = tmp_path/'camera mesh.stl'
    filename.write_text('solid t\nvertex 0 0 0\nvertex 1 1 1\nendsolid t\n')
    robot = model_fixture()
    geometry = ET.SubElement(ET.SubElement(robot.find('link'), 'collision'), 'geometry')
    ET.SubElement(geometry, 'mesh', filename=filename.as_uri())
    xml, _ = prepare_model(ET.tostring(robot), '', lambda _: tmp_path)
    assert ET.fromstring(xml).find('./link/collision/geometry/box') is not None


def test_continuous_goal_limits_per_frame_translation_and_rotation():
    current = np.eye(4)
    desired = mount_matrix([1., 0., 0.], [0., 0., 90.])
    result = bounded_goal(current, desired, .01, .05)
    from scipy.spatial.transform import Rotation
    assert result[:3, 3] == pytest.approx([.01, 0., 0.])
    assert Rotation.from_matrix(result[:3, :3]).magnitude() == pytest.approx(.05)
    assert np.allclose(bounded_goal(current, desired, 2., math.pi), desired)


@pytest.fixture
def bridge():
    rclpy = pytest.importorskip('rclpy')
    from ffw_teleop.sharpa_vr_bridge import SharpaVRBridge
    rclpy.init(domain_id=98)
    n = SharpaVRBridge()
    yield n
    n.stop('Test shutdown')
    n.destroy_node()
    rclpy.shutdown()


def feed_bridge(n):
    from geometry_msgs.msg import PoseStamped
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Float32
    for side in ('left', 'right'):
        n.on_grip(Float32(data=1.0), side)
        for kind, x in (('vr', .5), ('robot', .4)):
            msg = PoseStamped()
            msg.header.stamp = n.get_clock().now().to_msg()
            msg.header.frame_id = 'base_link'
            msg.pose.orientation.w = 1.0
            msg.pose.position.x = x
            n.on_pose(msg, side, kind)
    names = n.names('left') + n.names('right')
    n.on_joints(JointState(name=names, position=[.1]*14))


def test_deadman_timeout_and_no_automatic_reactivation(bridge):
    from std_msgs.msg import Float32
    n = bridge
    assert not n.enable()[0]
    feed_bridge(n)
    assert n.enable()[0]
    n.on_grip(Float32(data=0.), 'left')
    assert not n.active
    feed_bridge(n)
    n.tick()
    assert not n.active
    assert n.enable()[0]
    old = n.poses[('vr', 'right')]
    n.poses[('vr', 'right')] = (old[0], time.monotonic()-1, old[2])
    n.tick()
    assert not n.active


def test_glove_mount_mode_needs_no_grip_but_still_stops_on_pose_loss(bridge):
    from std_msgs.msg import Float32
    n = bridge
    n.p['require_squeeze'] = False
    feed_bridge(n)
    n.grips.clear()
    assert n.enable()[0]
    n.on_grip(Float32(data=0.0), 'left')
    n.tick()
    assert n.active
    sample = n.poses[('vr', 'left')]
    n.poses[('vr', 'left')] = (sample[0], time.monotonic()-1, sample[2])
    n.tick()
    assert not n.active


def test_arm_gate_rejects_finger_commands(bridge, monkeypatch):
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
    n = bridge
    sent = []
    monkeypatch.setattr(n, 'publish_arm', lambda side, values: sent.append((side, list(values))))
    feed_bridge(n)
    assert n.enable()[0]
    msg = JointTrajectory(joint_names=n.names('left'),
                          points=[JointTrajectoryPoint(positions=[.2]*7)])
    n.on_trajectory(msg, 'left')
    assert sent[-1] == ('left', [.2]*7)
    msg.joint_names.append('left_index_MCP_FE')
    msg.points[0].positions.append(.3)
    n.on_trajectory(msg, 'left')
    assert not n.active
    assert all(len(values)==7 for _, values in sent)


def test_pose_jump_and_invalid_passive_feedback_stop_arms(bridge):
    from geometry_msgs.msg import PoseStamped
    from sensor_msgs.msg import JointState
    n = bridge
    feed_bridge(n)
    assert n.enable()[0]
    msg = PoseStamped()
    msg.header.stamp = n.get_clock().now().to_msg()
    msg.header.frame_id = 'base_link'
    msg.pose.orientation.w = 1.0
    msg.pose.position.x = 3.0
    n.on_pose(msg, 'left', 'vr')
    assert not n.active
    feed_bridge(n)
    assert n.enable()[0]
    n.on_joints(JointState(name=['left_index_MCP_FE'], position=[float('nan')]))
    assert not n.active and not n.feedback


def test_ik_divergence_and_silent_output_are_reported_as_stopped(bridge):
    from std_msgs.msg import Bool
    n = bridge
    feed_bridge(n)
    assert n.enable()[0]
    n.on_divergence(Bool(data=True))
    assert n.active  # anchor reset is ignored during the vendor startup delay
    n.armed_at -= 4
    n.on_divergence(Bool(data=True))
    assert not n.active
    feed_bridge(n)
    assert n.enable()[0]
    n.last_ik['left'] = time.monotonic()-2
    n.tick()
    assert not n.active and n.last_stop_reason == 'Cyclo arm output stopped'


def test_mount_calibration_removes_neutral_inward_tilt_and_keeps_arms_stopped(bridge, tmp_path):
    from std_srvs.srv import Trigger
    import yaml
    n = bridge
    feed_bridge(n)
    n.p['calibration_file'] = str(tmp_path/'mount.yaml')
    # A controller mounted 30 degrees inward at the operator's neutral pose.
    old = n.poses[('vr', 'left')]
    n.poses[('vr', 'left')] = (mount_matrix([.5, 0., 0.], [30., 0., 0.]), old[1], old[2])
    response = n.on_calibrate_mount(Trigger.Request(), Trigger.Response())
    assert response.success and not n.active
    assert n.p['left_mount_rpy_deg'] == pytest.approx([-30., 0., 0.])
    assert np.allclose(n.poses[('vr', 'left')][0][:3, :3], np.eye(3))
    saved = yaml.safe_load((tmp_path/'mount.yaml').read_text())
    assert saved['sharpa_vr_bridge']['ros__parameters']['left_mount_rpy_deg'] == pytest.approx([-30., 0., 0.])
    assert n.enable()[0]
    assert np.allclose(n.filtered_goals['left'][:3, :3], np.eye(3))
