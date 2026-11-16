from types import SimpleNamespace
import time

import rclpy
from sensor_msgs.msg import JointState

from ffw_teleop.lift_keyboard import ArrowKeys, LiftKeyboard
from ffw_teleop.record_rosbag import resolve_topics, current_interfaces
from ffw_teleop.recording_model import bundle_model, replay_model
from ffw_teleop.replay_rosbag import display_topics


def test_split_keys():
    keys = ArrowKeys()
    assert keys.feed(b'\x1b[') == []
    assert keys.feed(b'A\x1b[B q') == ['up', 'down', 'hold', 'quit']


def test_inventory_selects_first_available_and_reports_missing():
    config = {'required': {'state': ['/a', '/b']},
              'cameras': {'left': {'depth': ['/d']}}, 'extra_topics': ['/tf']}
    selected, missing, topics = resolve_topics(config, {'/b': [], '/tf': []})
    assert selected == {'state': '/b'}
    assert missing == ['left/depth']
    assert topics == ['/b', '/tf']


def test_raw_current_not_effort():
    msg = SimpleNamespace(joint_names=['dxl1'], interface_values=[
        SimpleNamespace(interface_names=['effort', 'Present Current'], values=[5., 17.])])
    assert current_interfaces(msg) == {'dxl1': 17.}


def test_portable_model(tmp_path):
    mesh = tmp_path/'mesh.stl'
    mesh.write_bytes(b'example')
    xml = f'<robot name="test"><link name="base"><visual><geometry><mesh filename="{mesh.as_uri()}"/></geometry></visual></link><ros2_control name="x"/></robot>'
    bundle_model(xml, tmp_path/'model', lambda _: None)
    mesh.unlink()
    model = replay_model(tmp_path/'model')
    assert '__BAG_MODEL_ROOT__' not in model.read_text()
    assert 'ros2_control' not in model.read_text()
    assert (tmp_path/'model/meshes/000_mesh.stl').read_bytes() == b'example'


def test_rviz_whitelist_has_no_commands():
    topics = display_topics({'left/rgb': '/image', 'motor_interfaces': '/dynamic_joint_states',
                             'command': '/arm_r_controller/joint_trajectory'})
    assert '/image' in topics
    assert '/arm_r_controller/joint_trajectory' not in topics
    assert '/dynamic_joint_states' not in topics


def test_lift_bounds_freshness_and_measured_relative_steps():
    # Default context is used by LiftKeyboard; this test never joins robot domain.
    rclpy.init(domain_id=91)
    node = LiftKeyboard(dry_run=True)
    try:
        assert node.command(1) is None
        node.on_joints(JointState(name=['lift_joint'], position=[-.005]))
        assert node.command(1) == 0.
        node.last_command = -float('inf')
        assert node.command(1) == 0.
        node.on_joints(JointState(name=['lift_joint'], position=[-.499]))
        node.last_command = -float('inf')
        assert node.command(-1) == -.5
        node.feedback = (-.2, time.monotonic()-1.)
        assert node.command(0) is None
    finally:
        node.destroy_node()
        rclpy.shutdown()
