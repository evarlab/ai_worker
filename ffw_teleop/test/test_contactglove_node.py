"""Exercise watchdog and arming with real ROS messages and the official SDK models."""
import time

import pytest

rclpy = pytest.importorskip('rclpy')
sdk = pytest.importorskip('contactglovesdk')
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool
from ffw_teleop.contactglove_sharpa import ContactGloveSharpa


@pytest.fixture
def node(monkeypatch):
    # Never connect unit tests to a user's running DivingStation.
    monkeypatch.setattr(sdk.ContactGloveClient, 'connect_to_host', lambda *args: None)
    monkeypatch.setattr(sdk.ContactGloveClient, 'disconnect', lambda *args: None)
    rclpy.init(domain_id=99)
    n = ContactGloveSharpa()
    yield n
    n.close()
    n.destroy_node()
    rclpy.shutdown()


def feed(node, timestamp=None, tracking=0):
    node.on_joints(JointState(name=node.names('right'), position=[0.2] * 15))
    node.on_frame(sdk.ErgonomicsData(id='test-right', is_left=False,
        timestamp_ms=timestamp if timestamp is not None else int(time.time() * 1000),
        tracking_state=tracking, values=[40.] * 20))


def enable(node):
    return node.on_enable(SetBool.Request(data=True), SetBool.Response())


def test_arm_slew_timeout_requires_rearm(node):
    assert not enable(node).success
    feed(node)
    assert enable(node).success
    assert node.command['right'] == [0.2] * 15
    sent = []
    node.publish = lambda side, values: sent.append((side, list(values)))
    node.last_tick -= 0.03
    node.tick()
    assert all(0.2 <= x <= 0.2 + 0.5 / 30 for x in sent[-1][1])
    stamp = node.frames['right'][2]
    old = node.frames['right']
    node.frames['right'] = (time.monotonic() - 1., *old[1:])
    feed(node, timestamp=stamp)  # a replay must not refresh the watchdog
    node.tick()
    assert not node.enabled
    count = len(sent)
    feed(node, timestamp=stamp + 1)
    node.tick()
    assert len(sent) == count  # recovered data alone cannot resume
    assert enable(node).success


def test_invalid_tracking_and_old_timestamp(node):
    feed(node, tracking=sdk.TrackingState.IkFailed)
    assert not enable(node).success
    feed(node, timestamp=int((time.time() - 3) * 1000))
    assert not enable(node).success
    feed(node)
    assert enable(node).success
    node.on_enable(SetBool.Request(data=False), SetBool.Response())
    assert not node.enabled


def test_skipped_ik_does_not_refresh_or_replace_valid_pose(node):
    feed(node)
    previous = node.frames['right']
    feed(node, timestamp=previous[2] + 1, tracking=sdk.TrackingState.NotAttempted)
    assert node.frames['right'] == previous


def test_four_fingers_leave_thumb_out_of_commands(node):
    node.indices = list(range(3, 15))
    feed(node)
    assert len(node.names('right')) == 12
    assert all('thumb' not in name for name in node.names('right'))
    assert len(node.frames['right'][3]) == 12
    assert enable(node).success
    assert len(node.command['right']) == 12


def test_stop_replaces_unreached_goal_with_measured_pose(node, monkeypatch):
    feed(node)
    assert enable(node).success
    node.command['right'] = [0.8]*15
    sent = []
    monkeypatch.setattr(node, 'publish', lambda side, values: sent.append(list(values)))
    node.stop()
    assert sent == [[0.2]*15]
    assert not node.enabled


def test_ros_bridge_auto_start_stop_and_relaunch(monkeypatch):
    messages = pytest.importorskip('contact_glove_msgs.msg')
    rclpy.init(domain_id=99, args=['--ros-args', '-p', 'input_mode:=ros',
                                 '-p', 'auto_enable:=true', '-p', 'max_speed:=0.0',
                                 '-p', 'clamp_targets:=false'])
    n = ContactGloveSharpa()
    try:
        present = [True]
        monkeypatch.setattr(n, 'count_publishers', lambda _: int(present[0]))
        sent = []
        monkeypatch.setattr(n, 'publish', lambda side, values: sent.append(list(values)))
        msg = messages.Ergonomics(id='test-right', is_left=False,
                                  generated_at_ms=int(time.time() * 1000))
        msg.index.mcp = 90.0
        n.on_ros_frame(msg, 'right')
        n.on_joints(JointState(name=n.names('right'), position=[0.2] * 15))
        n.tick()
        assert n.enabled
        assert sent[-1][3] == pytest.approx(3.141592653589793 / 2)
        # Stop the source, then a stale cache may not restart motion.
        present[0] = False
        n.tick()
        assert not n.enabled and not n.frames
        present[0] = True
        n.tick()
        assert not n.enabled
        msg.generated_at_ms += 1
        n.on_ros_frame(msg, 'right')
        n.tick()
        assert n.enabled
        # A tracking failure latches off until the bridge is restarted/rearmed.
        msg.generated_at_ms += 1
        msg.tracking_state = 1
        n.on_ros_frame(msg, 'right')
        n.tick()
        assert not n.enabled
        msg.generated_at_ms += 1
        msg.tracking_state = 0
        n.on_ros_frame(msg, 'right')
        n.tick()
        assert not n.enabled
    finally:
        n.close()
        n.destroy_node()
        rclpy.shutdown()


def test_abduction_adds_five_lateral_joints(monkeypatch):
    monkeypatch.setattr(sdk.ContactGloveClient, 'connect_to_host', lambda *args: None)
    monkeypatch.setattr(sdk.ContactGloveClient, 'disconnect', lambda *args: None)
    rclpy.init(domain_id=99, args=['--ros-args', '-p', 'include_abduction:=true',
                                 '-p', 'clamp_targets:=false'])
    n = ContactGloveSharpa()
    try:
        frame = sdk.ErgonomicsData(id='test-right', is_left=False,
                                  timestamp_ms=int(time.time()*1000), values=[0.0]*20)
        frame.values[7] = -10.0
        n.on_frame(frame)
        assert len(n.names('right')) == 20
        goal = dict(zip(n.names('right'), n.frames['right'][3]))
        assert goal['right_index_MCP_AA'] == pytest.approx(-3.141592653589793/18)
        assert 'right_thumb_MCP_AA' in goal
        assert 'right_thumb_CMC_AA' not in goal
        assert 'right_pinky_CMC' not in goal
    finally:
        n.close()
        n.destroy_node()
        rclpy.shutdown()


def test_manual_gate_reports_missing_input_then_enables_from_feedback(node):
    from std_srvs.srv import Trigger
    node.auto_arm_pending=False
    status=node.on_status(Trigger.Request(),Trigger.Response())
    assert not status.success and not node.enabled
    assert 'glove' in status.message
    feed(node)
    assert not node.enabled
    status=node.on_status(Trigger.Request(),Trigger.Response())
    assert status.success and not node.enabled
    assert enable(node).success and node.enabled


def test_hardware_hold_blocks_enable_and_stops_tracking(node):
    from diagnostic_msgs.msg import DiagnosticArray,DiagnosticStatus,KeyValue
    feed(node)
    node.p['require_hardware_diagnostics']=True
    assert not enable(node).success
    assert 'hardware' in node.wait_reason
    healthy=DiagnosticStatus(name='sharpa_wave: right hand',level=DiagnosticStatus.OK,message='enabled',values=[KeyValue(key='ros2_control_active',value='true')])
    node.on_diagnostics(DiagnosticArray(status=[healthy]))
    assert enable(node).success
    held=DiagnosticStatus(name='sharpa_wave: right hand',level=DiagnosticStatus.WARN,message='holding: joint limit',values=[KeyValue(key='ros2_control_active',value='true')])
    node.on_diagnostics(DiagnosticArray(status=[held]));node.tick()
    assert not node.enabled
    assert not enable(node).success and 'holding' in node.wait_reason
