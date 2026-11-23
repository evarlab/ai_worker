import signal
from types import SimpleNamespace

import pytest

from ffw_teleop.foot_pedal import PedalEdges, PedalActions
from ffw_teleop import recording_control


def test_pedal_edges_ignore_hold_repeat_and_bounce():
    keys = PedalEdges()
    assert keys.feed('pedal', 30, 1, 1.) == 'teleop'
    assert keys.feed('pedal', 30, 2, 2.) is None
    assert keys.feed('pedal', 30, 1, 2.) is None
    assert keys.feed('pedal', 30, 0, 2.) is None
    assert keys.feed('pedal', 30, 1, 2.1) == 'teleop'
    keys.feed('pedal', 30, 0, 2.15)
    assert keys.feed('pedal', 30, 1, 2.2) is None
    assert keys.feed('pedal', 48, 1, 2.2) == 'record'
    assert keys.feed('pedal', 46, 1, 2.2) == 'home'
    assert keys.feed('normal keyboard', 99, 1, 3.) is None


class Process:
    def __init__(self):
        self.returncode = None
        self.stdin = None

    def poll(self):
        return self.returncode


@pytest.fixture
def actions(tmp_path, monkeypatch):
    manager = PedalActions(tmp_path, tmp_path)
    manager.generation_file = tmp_path/'generation'
    manager.generation = ''
    manager.commands = []
    manager.interrupted = []
    def launch(*command):
        process = Process()
        manager.commands.append((command, process))
        return process
    monkeypatch.setattr(manager, 'launch', launch)
    monkeypatch.setattr(manager, 'following_service', lambda: False)
    monkeypatch.setattr(manager, 'interrupt', manager.interrupted.append)
    return manager


def test_second_left_stops_during_start(actions):
    actions.press('teleop')
    start = actions.motion
    assert actions.commands[-1][0] == ('resume_teleop.sh',)
    actions.press('teleop')
    assert actions.commands[-1][0] == ('stop_teleop.sh',)
    assert start in actions.interrupted


def test_successful_start_toggles_stop(actions):
    actions.press('teleop')
    actions.motion.returncode = 0
    actions.poll()
    assert actions.following
    actions.press('teleop')
    assert actions.commands[-1][0] == ('stop_teleop.sh',)


def test_home_waits_for_stop_and_does_not_resume(actions):
    actions.press('teleop')
    actions.press('home')
    assert actions.commands[-1][0] == ('stop_teleop.sh',)
    actions.motion.returncode = 0
    actions.poll()
    assert actions.commands[-1][0] == ('home_teleop.sh',)
    actions.motion.returncode = 0
    actions.poll()
    assert not actions.following


def test_recording_is_independent_and_gracefully_stops(actions):
    actions.press('record')
    assert actions.commands[-1][0] == ('record_rosbag.sh', '--toggle')
    assert actions.motion is None
    actions.press('record')
    assert actions.commands[-1][0] == ('record_rosbag.sh', '--stop')
    count = len(actions.commands)
    actions.press('record')
    assert len(actions.commands) == count


def test_dry_run_never_launches(actions):
    actions.dry_run = True
    for action in ('teleop', 'record', 'home'):
        actions.press(action)
    actions.shutdown()
    assert not actions.commands


def test_recorder_lock_excludes_duplicates_and_cleans_up(tmp_path, monkeypatch):
    monkeypatch.setattr(recording_control, 'control_path', lambda _: tmp_path/'recorder.json')
    lock = recording_control.RecorderLock(93)
    try:
        assert recording_control.recording_active(93)
        with pytest.raises(RuntimeError, match='既に記録中'):
            recording_control.RecorderLock(93)
    finally:
        lock.close()
    assert not recording_control.recording_active(93)


def test_stale_recorder_is_never_signalled(tmp_path, monkeypatch):
    path = tmp_path/'recorder.json'
    path.write_text('{"pid": 123, "identity": "old"}')
    monkeypatch.setattr(recording_control, 'control_path', lambda _: path)
    monkeypatch.setattr(recording_control, 'process_identity', lambda _: 'new')
    monkeypatch.setattr(recording_control.os, 'kill', lambda *_: pytest.fail('stale pid was signalled'))
    assert not recording_control.recording_active(93)
    assert recording_control.stop_recording(93) == 0


def test_human_pedals_only_use_collector_and_dry_run_is_inert():
    from ffw_teleop.foot_pedal import HumanPedalActions
    calls=[]
    def request(data):
        calls.append(data)
        return {'result':None,'status':{'following':True,'recording':False}}
    manager=HumanPedalActions('pick', request=request)
    for action in ('teleop','record','home'):manager.press(action)
    assert calls==[{'action':'follow-toggle'},
                   {'action':'toggle','task':'pick','outcome':'success'},
                   {'action':'stop','outcome':'failure'}]
    manager.shutdown()
    assert calls[-1]=={'action':'follow-stop'}
    calls.clear()
    manager=HumanPedalActions(dry_run=True,request=request)
    for action in ('teleop','record','home'):manager.press(action)
    manager.shutdown()
    assert not calls
