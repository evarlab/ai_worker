import asyncio
import gzip
import json
from types import SimpleNamespace

import numpy as np
import pytest

from ffw_teleop.human_demo_data import HumanEpisodes, rigid, task_frame
from ffw_teleop.export_human_demo import export_episode


def pose(x=0., y=0., z=0.):
    matrix = np.eye(4)
    matrix[:3, 3] = [x, y, z]
    return matrix.reshape(-1, order='F').tolist()


def test_task_coordinates_and_no_reflections():
    frame = task_frame([0, 1, 0], [.2, 1, 0], [0, 1, -.2])
    assert np.allclose(frame[:3, :3], [[1, 0, 0], [0, 0, 1], [0, -1, 0]])
    assert np.linalg.det(frame[:3, :3]) == pytest.approx(1.)
    with pytest.raises(ValueError):
        task_frame([0, 0, 0], [.2, 0, 0], [0, 0, .2])
    with pytest.raises(ValueError):
        task_frame([0, 0, 0], [.2, 0, 0], [.4, 0, 0])


def test_rigid_rejects_reflection():
    matrix = np.eye(4)
    matrix[0, 0] = -1
    with pytest.raises(ValueError):
        rigid(matrix)


def test_tracking_status_reports_behind_head_and_rejects_emulation(tmp_path):
    episodes = HumanEpisodes(tmp_path)
    episodes.record('xr', dict(reference_id='one', head=pose(), left=pose(0, 0, .3),
        right=pose(0, 0, -.3), controllers={'left': {'profiles': ['touch-pro']},
                                          'right': {'emulated_position': True}}))
    status = episodes.status()
    assert status['tracking']['left']['tracked']
    assert status['tracking']['left']['behind_head']
    assert not status['tracking']['right']['tracked']
    assert status['tracking']['left']['profiles'] == ['touch-pro']
    assert status['xr_received'] == 1
    episodes.latest['xr']['receive_monotonic_ns'] -= 1_000_000_000
    assert not episodes.status()['tracking']['left']['tracked']


def test_marks_and_reference_reset(tmp_path):
    episodes = HumanEpisodes(tmp_path)
    episodes.record('reference', dict(reference_id='one'))
    for name, xyz in [('origin', [0, 1, 0]), ('right', [.2, 1, 0]), ('forward', [0, 1, -.2])]:
        episodes.record('xr', dict(reference_id='one', right=pose(*xyz)))
        episodes.mark(name)
    assert episodes.calibration
    episodes.start('pick')
    with pytest.raises(ValueError, match='収集中'):
        episodes.mark('origin')
    episodes.record('reference', dict(reference_id='two'))
    assert not episodes.calibration
    episodes.stop('success')


def test_export_preserves_reset_gaps_and_camera_payload(tmp_path):
    episodes = HumanEpisodes(tmp_path)
    episodes.record('reference', dict(reference_id='one'))
    directory = episodes.start('pick')
    metadata = dict(reference_id='one', source='quest_passthrough_browser')
    jpeg = b'\xff\xd8test-image\xff\xd9'
    episodes.image('quest_rgb', jpeg, metadata)
    for side in ('left', 'right'):
        episodes.record('glove_'+side, dict(source_timestamp_ms=123, angles_degrees=list(range(20))))
    episodes.record('xr', dict(reference_id='one', left=pose(), right=pose(1), head=pose(0, 1)))
    episodes.record('xr', dict(reference_id='one', left=pose(.01), right=None, head=pose(0, 1)))
    episodes.record('reference', dict(reference_id='two'))
    episodes.record('xr', dict(reference_id='two', left=pose(100), right=pose(), head=pose(0, 1)))
    episodes.stop('success')
    output = export_episode(directory)
    arrays = np.load(output)
    assert arrays['pose_valid'].tolist() == [[True, True], [True, False], [True, True]]
    assert arrays['delta_valid'].tolist() == [[False, False], [True, False], [False, False]]
    assert not arrays['task_valid'].any()
    assert not arrays['image_valid'][-1]
    assert arrays['glove_angles_degrees'][0, 0].tolist() == list(range(20))
    image = next((output.parent/'images/quest_rgb').glob('*.jpg'))
    assert image.read_bytes() == jpeg
    assert json.loads((output.parent/'manifest.json').read_text())['complete']


def test_incomplete_episode_not_exported(tmp_path):
    episodes = HumanEpisodes(tmp_path)
    directory = episodes.start('test')
    try:
        with pytest.raises(ValueError, match='未完了'):
            export_episode(directory)
    finally:
        episodes.stop()


def test_glove_duplicate_and_invalid_never_refresh_readiness(tmp_path):
    pytest.importorskip('aiohttp')
    from ffw_teleop.human_demo import glove_event
    episodes = HumanEpisodes(tmp_path)
    frame = SimpleNamespace(id='left', is_left=True, values=[1.]*20, timestamp_ms=123, tracking_state=0)
    glove_event(episodes, frame)
    first = episodes.latest['glove_left']
    glove_event(episodes, frame)
    assert episodes.latest['glove_left'] is first
    frame.timestamp_ms = 124
    frame.tracking_state = 2
    glove_event(episodes, frame)
    assert episodes.latest['glove_left'] is first


def test_http_websocket_camera_episode_roundtrip(tmp_path):
    pytest.importorskip('aiohttp')
    from aiohttp.test_utils import TestClient, TestServer
    from ffw_teleop.human_demo import application
    async def run():
        episodes = HumanEpisodes(tmp_path)
        client = TestClient(TestServer(application(episodes, 'secret')))
        await client.start_server()
        auth = {'Authorization': 'Bearer secret'}
        try:
            response = await client.get('/status')
            assert response.status == 401
            response = await client.post('/control', headers=auth, json=dict(action='start', task='pick'))
            assert response.status == 400
            ws = await client.ws_connect('/events?token=secret')
            await ws.send_json(dict(event='reference', data=dict(reference_id='one')))
            await ws.send_json(dict(event='xr', data=dict(reference_id='one', left=pose(), right=pose(1), head=pose(0, 1))))
            # Ping is ordered after poses and confirms the server consumed them.
            await ws.send_json(dict(event='ping', data=dict(client_ms=1.)))
            assert (await ws.receive_json())['event'] == 'pong'
            for side in ('left', 'right'):
                episodes.record('glove_'+side, dict(source_timestamp_ms=123, angles_degrees=[1.]*20))
            image_headers = {**auth, 'X-Frame-Metadata': json.dumps(dict(source='quest_passthrough_browser', verified_environment=True, reference_id='one'))}
            response = await client.post('/camera/quest_rgb', headers=image_headers, data=b'\xff\xd8frame\xff\xd9')
            assert response.status == 200
            response = await client.post('/control', headers=auth, json=dict(action='start', task='pick'))
            assert response.status == 200, await response.text()
            response = await client.post('/camera/quest_rgb', headers=image_headers, data=b'\xff\xd8frame\xff\xd9')
            assert response.status == 200
            await ws.send_json(dict(event='xr', data=dict(reference_id='one', left=pose(), right=pose(), head=pose())))
            await ws.send_json(dict(event='ping', data=dict(client_ms=2.)))
            await ws.receive_json()
            response = await client.post('/control', headers=auth, json=dict(action='stop', outcome='success'))
            assert response.status == 200
            await ws.close()
            directory = episodes.directory
            assert json.loads((directory/'manifest.json').read_text())['camera_streams']
            assert len(list((directory/'images/quest_rgb').glob('*.jpg'))) == 1
            assert export_episode(directory).exists()
        finally:
            await client.close()
    asyncio.run(run())


def test_trim_preserves_causal_gloves_and_resets_first_delta(tmp_path):
    directory = tmp_path
    (directory/'manifest.json').write_text(json.dumps(dict(complete=True)))
    events = []
    for i in range(41):
        for side in ('left', 'right'):
            events.append(dict(event='glove_'+side, receive_monotonic_ns=i*10**9,
                data=dict(source_timestamp_ms=i*1000, angles_degrees=[float(i)]*20)))
        events.append(dict(event='xr', receive_monotonic_ns=i*10**9,
            data=dict(reference_id='one', left=pose(i/100), right=pose(), head=pose())))
    with gzip.open(directory/'events.jsonl.gz','wt') as f:
        for e in events: f.write(json.dumps(e)+'\n')
    original = (directory/'events.jsonl.gz').read_bytes()
    output = export_episode(directory, trim_start=10, trim_end=10)
    arrays = np.load(output)
    assert arrays['receive_monotonic_ns'].tolist() == [i*10**9 for i in range(10,31)]
    assert arrays['glove_angles_degrees'][0,0,0] == 10
    assert not arrays['delta_valid'][0].any()
    assert (directory/'events.jsonl.gz').read_bytes() == original
    with pytest.raises(ValueError,match='残りません'): export_episode(directory,trim_start=30,trim_end=30)


def test_offline_arm_fk_respects_urdf_origins_and_joint_axes():
    import xml.etree.ElementTree as ET
    from ffw_teleop.human_demo_sim import Arm
    root = ET.Element('robot')
    for i in range(1,8):
        j = ET.SubElement(root,'joint',name=f'arm_l_joint{i}',type='revolute')
        ET.SubElement(j,'parent',link='base_link' if i==1 else f'arm_l_link{i-1}')
        ET.SubElement(j,'child',link=f'arm_l_link{i}')
        ET.SubElement(j,'origin',xyz='1 0 0')
        ET.SubElement(j,'axis',xyz='0 0 1')
        ET.SubElement(j,'limit',lower='-3.14',upper='3.14')
    arm = Arm(root,'l')
    assert arm.fk(np.zeros(7))[:3,3] == pytest.approx([7,0,0])
    assert arm.fk(np.array([np.pi/2]+[0.]*6))[:3,3] == pytest.approx([1,6,0])
