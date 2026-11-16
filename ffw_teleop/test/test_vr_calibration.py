import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from ffw_teleop.vr_calibration_math import average_pose, fit_motion, validate_translation
from ffw_teleop.vr_pose_math import mount_matrix, relative_goal
from ffw_teleop.sharpa_vuer_publisher import WorldReferenceMixin


def fixture_motion():
    axes = Rotation.from_euler('xyz', [12., -8., 35.], degrees=True).as_matrix()
    neutral = mount_matrix([.5, .3, 1.], [20., 5., -15.])
    lever = np.array([.04, -.08, -.03])
    translations, rotations = [], []
    for i in range(3):
        endpoint = neutral.copy()
        endpoint[:3, 3] += axes.T[:, i]*.1
        translations.append((neutral.copy(), endpoint))
        endpoint = neutral.copy()
        delta = Rotation.from_rotvec(axes.T[:, i]*np.deg2rad(45)).as_matrix()
        endpoint[:3, :3] = delta @ neutral[:3, :3]
        endpoint[:3, 3] += (neutral[:3, :3]-endpoint[:3, :3]) @ lever
        rotations.append((neutral.copy(), endpoint))
    return axes, lever, translations, rotations


def test_fit_recovers_all_three_axes_and_rotation_pivot():
    axes, lever, translations, rotations = fixture_motion()
    fit = fit_motion(translations, rotations)
    recovered = Rotation.from_euler('xyz', fit['axes_rpy_deg'], degrees=True).as_matrix()
    assert np.allclose(recovered, axes)
    assert fit['mount_xyz'] == pytest.approx(lever)
    assert fit['position_scale'] == pytest.approx(1.)
    assert fit['pivot_residual_m'] < 1e-12
    # Translation and rotation must use the same world->robot correction.
    robot = mount_matrix([.1, .2, .3], [2., 8., 5.])
    for i, (initial, current) in enumerate(translations):
        goal = relative_goal(current, initial, robot, 1., recovered)
        expected = robot[:3, 3].copy()
        expected[i] += .1
        assert goal[:3, 3] == pytest.approx(expected)
    for i, (initial, current) in enumerate(rotations):
        goal = relative_goal(current, initial, robot, 1., recovered)
        expected_delta = Rotation.from_rotvec(np.eye(3)[i]*np.deg2rad(45)).as_matrix()
        assert np.allclose(goal[:3, :3], expected_delta @ robot[:3, :3])
        mount = mount_matrix(fit['mount_xyz'], [0., 0., 0.])
        corrected = relative_goal(current @ mount, initial @ mount, robot, 1., recovered)
        assert corrected[:3, 3] == pytest.approx(robot[:3, 3])


def test_fit_rejects_mirrored_or_repeated_axes_and_bad_pivot():
    _, _, translations, rotations = fixture_motion()
    a, b = translations[1]
    mirrored = b.copy()
    mirrored[:3, 3] = 2*a[:3, 3]-b[:3, 3]
    with pytest.raises(ValueError, match='符号'):
        fit_motion([translations[0], (a, mirrored), translations[2]], rotations)
    fit = fit_motion(translations, [rotations[0]]*3)
    assert not fit['pivot_applied'] and fit['warnings']
    assert fit['mount_xyz'] == [0., 0., 0.]
    bad = [(a.copy(), b.copy()) for a, b in rotations]
    bad[0][1][:3, 3] += [.12, .12, .12]
    fit = fit_motion(translations, bad)
    assert not fit['pivot_applied'] and fit['warnings']


def test_axis_fit_ignores_different_translation_distances():
    axes, _, translations, rotations = fixture_motion()
    varied = []
    for (a, b), scale in zip(translations, [.45, 1.7, 3.2]):
        endpoint = b.copy()
        endpoint[:3, 3] = a[:3, 3] + scale*(b[:3, 3]-a[:3, 3])
        varied.append((a, endpoint))
    fit = fit_motion(varied, rotations)
    assert fit['position_scale'] == 1.0
    assert np.allclose(Rotation.from_euler('xyz', fit['axes_rpy_deg'], degrees=True).as_matrix(), axes)
    assert fit['translation_lengths_m'] == pytest.approx([.045, .17, .32])


def test_manual_translation_axes_are_orthogonalized_with_warning():
    _, _, translations, rotations = fixture_motion()
    directions = np.array([[1., .37, .086], [0., .929, .376], [0., 0., .923]])
    varied = []
    for i, (a, b) in enumerate(translations):
        endpoint = b.copy()
        endpoint[:3, 3] = a[:3, 3] + .2*directions[:, i]
        varied.append((a, endpoint))
    fit = fit_motion(varied, rotations)
    assert fit['warnings']
    axes = Rotation.from_euler('xyz', fit['axes_rpy_deg'], degrees=True).as_matrix()
    assert np.allclose(axes @ axes.T, np.eye(3))
    assert np.linalg.det(axes) == pytest.approx(1.)


def test_direction_calibration_allows_wrist_rotation_during_translation():
    initial = mount_matrix([0., 0., 0.], [0., 0., 0.])
    endpoint = mount_matrix([.15, 0., 0.], [0., 35., 0.])
    assert np.allclose(validate_translation(initial, endpoint), [.15, 0., 0.])
    endpoint = mount_matrix([.15, 0., 0.], [0., 15., 0.])
    assert np.allclose(validate_translation(initial, endpoint), [.15, 0., 0.])


def test_cancel_wizard_saves_audit_without_double_shutdown(tmp_path):
    import os
    import signal
    import subprocess
    import sys
    import time
    env = dict(os.environ, HOME=str(tmp_path), ROS_DOMAIN_ID='94')
    process = subprocess.Popen([sys.executable, '-u', '-c',
        'import sys; from ffw_teleop.calibrate_vr import main; sys.exit(main())'],
        stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, env=env, text=True)
    try:
        # Wait for the first input prompt without writing any poses or commands.
        import select
        output = ''
        end = time.monotonic()+10
        while '左手' not in output and time.monotonic() < end:
            if select.select([process.stdout], [], [], .1)[0]:
                output += os.read(process.stdout.fileno(), 4096).decode()
        assert '左手' in output
        process.send_signal(signal.SIGINT)
        rest, _ = process.communicate(timeout=5)
        assert process.returncode == 130, output+rest
        assert 'Traceback' not in output+rest
        audits = list((tmp_path/'.local/state/ai_worker/calibration').glob('poses-*.jsonl'))
        assert audits and 'cancelled' in audits[0].read_text()
        assert not (tmp_path/'.config/ai_worker/vr_motion_calibration.yaml').exists()
    finally:
        if process.poll() is None:
            process.kill()
            process.wait()


def test_average_tolerates_hand_jitter_and_position_outliers():
    samples = [mount_matrix([.1, .2, .3], [0., 0., 179.]) for _ in range(10)]
    samples[-1][:3, :3] = Rotation.from_euler('z', -179., degrees=True).as_matrix()
    assert np.isfinite(average_pose(samples)).all()
    samples[-1][:3, 3] += [.05, 0., 0.]
    assert average_pose(samples)[:3, 3] == pytest.approx([.1, .2, .3])


def test_headset_movement_does_not_change_world_controller_pose():
    class Upstream:
        def _publish_synced_pose_frame_if_ready(self):
            assert self.pending_body_pose_frame
            return self.head_inverse_matrix @ self.controller

    class Publisher(WorldReferenceMixin, Upstream):
        pass

    publisher = Publisher()
    publisher.controller = mount_matrix([.5, .2, .3], [10., 20., 30.])
    for head in ([0., 0., 0.], [45., 80., 120.]):
        publisher.pending_body_pose_frame = False
        publisher.head_inverse_matrix = np.linalg.inv(mount_matrix([.3, -.2, .1], head))
        assert np.allclose(publisher._publish_synced_pose_frame_if_ready(), publisher.controller)


def test_failed_hand_recording_can_resume_with_optional_pivot(tmp_path):
    import json
    from ffw_teleop.calibrate_vr import recover_recordings
    _, _, translations, rotations = fixture_motion()
    event = dict(event='fit_rejected', side='left', reason='old strict pivot check',
        translations=[[a.tolist(), b.tolist()] for a, b in translations],
        rotations=[[a.tolist(), b.tolist()] for a, b in [rotations[0]]*3])
    path = tmp_path/'poses.jsonl'
    path.write_text(json.dumps(event)+'\n')
    results, recordings = recover_recordings(path)
    assert set(results) == {'left'} and set(recordings) == {'left'}
    assert not results['left']['pivot_applied']
    assert results['left']['position_scale'] == 1.


def test_model_preview_resumes_after_jump_only_when_both_inputs_stabilize():
    from ffw_teleop.vr_preview import PreviewRecovery
    recovery = PreviewRecovery()
    pose = np.eye(4)
    for side in ('left', 'right'):
        recovery.observe(side, pose, 1.)
        recovery.observe(side, pose, 2.1)
    assert not recovery.ready(2.1)  # gap in input resets stability
    for t in np.arange(2.15, 3.3, .05):
        for side in ('left', 'right'):
            recovery.observe(side, pose, t)
    assert recovery.ready(3.3)
    recovery.requested = True
    recovery.status(True, 3.3)
    recovery.status(False, 3.4)
    jumped = pose.copy()
    jumped[0, 3] = 1.4
    recovery.observe('left', jumped, 3.4)
    assert not recovery.ready(3.5)
    for t in np.arange(3.45, 4.6, .05):
        recovery.observe('left', jumped, t)
        recovery.observe('right', pose, t)
    assert recovery.ready(4.6)
