import gzip
import json
import numpy as np

from ffw_teleop.teleop_recording import SessionRecorder


def test_session_recording_preserves_ros_pose_and_numpy_axes(tmp_path):
    from geometry_msgs.msg import PoseStamped
    recorder = SessionRecorder('test', tmp_path)
    pose = PoseStamped()
    pose.header.frame_id = 'base_link'
    pose.header.stamp.sec = 123
    pose.pose.position.x = .4
    pose.pose.orientation.w = 1.
    recorder.record('vr_pose_left', pose)
    recorder.record('anchor', {'axes': np.eye(3)})
    recorder.record('stop', {'reason': 'input stale'})
    recorder.close()
    recorder.close()
    with gzip.open(recorder.path, 'rt') as source:
        events = [json.loads(s) for s in source]
    assert [e['event'] for e in events] == ['session_start', 'vr_pose_left', 'anchor', 'stop']
    assert events[1]['data']['header']['stamp']['sec'] == 123
    assert events[1]['data']['pose']['position']['x'] == .4
    assert events[2]['data']['axes'] == np.eye(3).tolist()
    assert all('wall_time' in e and 'monotonic' in e for e in events)
