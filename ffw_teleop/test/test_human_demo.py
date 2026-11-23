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
            response = await client.post('/control', headers=auth, json=dict(action='follow-start'))
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


def test_aruco_real_jpeg_detection():
    cv2 = pytest.importorskip('cv2')
    from ffw_teleop.human_demo_aruco import ArucoDetector
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    marker = (cv2.aruco.generateImageMarker(dictionary, 7, 160)
              if hasattr(cv2.aruco, 'generateImageMarker') else cv2.aruco.drawMarker(dictionary, 7, 160))
    image = np.full((240, 320), 255, dtype=np.uint8)
    image[40:200, 80:240] = marker
    ok, jpeg = cv2.imencode('.jpg', image)
    assert ok
    result = ArucoDetector().detect(jpeg.tobytes())
    assert result['width'] == 320 and result['height'] == 240
    assert [m['id'] for m in result['markers']] == [7]
    assert np.allclose(result['markers'][0]['corners'], [[80,40],[239,40],[239,199],[80,199]], atol=2)
    _, blank = cv2.imencode('.jpg', np.full_like(image, 255))
    assert ArucoDetector().detect(blank.tobytes())['markers'] == []
    with pytest.raises(ValueError, match='decode'):
        ArucoDetector().detect(b'not a JPEG')


def test_terminal_controls_use_collector_and_eof_detaches(tmp_path, monkeypatch):
    import os
    import sys
    from ffw_teleop.human_demo import application, terminal_controls
    async def run():
        episodes = HumanEpisodes(tmp_path)
        app = application(episodes, 'secret', require_camera=False)
        app['episodes'] = episodes
        read_fd, write_fd = os.pipe()
        with os.fdopen(read_fd, 'r') as source:
            monkeypatch.setattr(sys, 'stdin', source)
            os.write(write_fd, b'start pick\nstatus\nstop success\nunknown\n')
            os.close(write_fd)
            stopped = asyncio.Event()
            await asyncio.wait_for(terminal_controls(app, stopped, 'demo'), 2.)
            assert not stopped.is_set()
            assert episodes.stream is None
        read_fd, write_fd = os.pipe()
        with os.fdopen(read_fd, 'r') as source:
            monkeypatch.setattr(sys, 'stdin', source)
            os.write(write_fd, b'quit\n')
            os.close(write_fd)
            await asyncio.wait_for(terminal_controls(app, stopped, 'demo'), 2.)
            assert stopped.is_set()
    asyncio.run(run())


def test_head_origin_and_current_head_hands_survive_export(tmp_path):
    e = HumanEpisodes(tmp_path)
    e.record('reference', dict(reference_id='one'))
    # Head initially faces +X: this catches using translations alone.
    initial = np.eye(4)
    initial[:3, :3] = [[0, 0, 1], [0, 1, 0], [-1, 0, 0]]
    initial[:3, 3] = [2, 1, 3]
    e.record('xr', dict(reference_id='one', head=initial.flatten(order='F').tolist()))
    e.follow_start()
    e.start('relative')
    current = initial.copy()
    current[:3, 3] += [1, 0, 0]
    hand = current.copy()
    hand[:3, 3] += [0, 0, -0.5]
    e.record('xr', dict(reference_id='one', head=current.flatten(order='F').tolist(),
        left=hand.flatten(order='F').tolist(), right=hand.flatten(order='F').tolist()))
    data = e.latest['xr']['data']
    assert np.allclose(np.array(data['T_head_origin_head'])[:3, 3], [0, 0, 1])
    assert np.allclose(np.array(data['T_head_left'])[:3, 3], [.5, 0, 0])
    with pytest.raises(ValueError, match='記録中'):
        e.follow_start()
    directory = e.directory
    e.follow_stop()
    assert not e.following and e.stream is None
    assert json.loads((directory/'manifest.json').read_text())['outcome'] == 'interrupted'
    arrays = np.load(export_episode(directory))
    assert arrays['head_origin_valid'].tolist() == [True]
    assert np.allclose(arrays['T_head_origin_head'][0, :3, 3], [0, 0, 1])
    assert np.allclose(arrays['T_head_controller'][0, 0, :3, 3], [.5, 0, 0])


def test_follow_requires_fresh_head_and_resets_on_reference_change(tmp_path):
    e = HumanEpisodes(tmp_path)
    with pytest.raises(ValueError, match='新しい頭'):
        e.follow_start()
    e.record('reference', dict(reference_id='one'))
    e.record('xr', dict(reference_id='one', head=pose()))
    e.latest['xr']['receive_monotonic_ns'] -= 1_000_000_000
    with pytest.raises(ValueError, match='新しい頭'):
        e.follow_start()
    e.record('xr', dict(reference_id='one', head=pose()))
    e.follow_start()
    e.start()
    e.record('reference', dict(reference_id='two'))
    assert not e.following and e.head_origin is None and e.stream is None


def test_human_pedal_api_toggles_and_saves_workspace(tmp_path):
    from aiohttp.test_utils import TestClient, TestServer
    from ffw_teleop.human_demo import application
    async def run():
        workspace={'simulation':{'table_surface_height_m':.7},'robot_deployment':{'calibrated':False}}
        e=HumanEpisodes(tmp_path,workspace=workspace)
        client=TestClient(TestServer(application(e,'secret',require_camera=False)))
        await client.start_server()
        async def action(name, **kw):
            response=await client.post('/control',headers={'Authorization':'Bearer secret'},json=dict(action=name,**kw))
            assert response.status==200,await response.text()
            return await response.json()
        try:
            e.record('reference',dict(reference_id='one'))
            e.record('xr',dict(reference_id='one',head=pose(),left=pose(),right=pose()))
            for side in ('left','right'):
                e.record('glove_'+side,dict(source_timestamp_ms=123,angles_degrees=[1.]*20))
            assert (await action('follow-toggle'))['status']['following']
            assert (await action('toggle',task='pick'))['status']['recording']
            result=await action('toggle',outcome='success')
            manifest=json.loads((e.directory/'manifest.json').read_text())
            assert manifest['outcome']=='success' and manifest['workspace']==workspace
            assert not result['status']['recording']
            assert result['status']['following']
            assert (await action('toggle',task='second'))['status']['recording']
            result=await action('follow-toggle')
            assert not result['status']['following'] and not result['status']['recording']
            assert json.loads((e.directory/'manifest.json').read_text())['outcome']=='interrupted'
        finally:await client.close()
    asyncio.run(run())


def test_camera_saves_jpeg_and_real_aruco_detection(tmp_path):
    cv2=pytest.importorskip('cv2')
    from aiohttp.test_utils import TestClient,TestServer
    from ffw_teleop.human_demo import application
    from ffw_teleop.human_demo_aruco import ArucoDetector
    async def run():
        e=HumanEpisodes(tmp_path)
        client=TestClient(TestServer(application(e,'secret',require_camera=False,detector=ArucoDetector())))
        await client.start_server()
        try:
            e.record('reference',dict(reference_id='one'))
            e.record('xr',dict(reference_id='one',head=pose(),left=pose(),right=pose()))
            e.follow_start();e.start('image_check')
            dictionary=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
            marker=(cv2.aruco.generateImageMarker(dictionary,4,160) if hasattr(cv2.aruco,'generateImageMarker') else cv2.aruco.drawMarker(dictionary,4,160))
            im=np.full((240,240),255,dtype=np.uint8);im[40:200,40:200]=marker
            _,jpeg=cv2.imencode('.jpg',im)
            metadata=dict(source='quest_passthrough_browser',verified_environment=True,reference_id='one',client_ms=123)
            response=await client.post('/camera/quest_rgb',data=jpeg.tobytes(),headers={
                'Authorization':'Bearer secret','X-Frame-Metadata':json.dumps(metadata)})
            assert response.status==200
            assert [m['id'] for m in (await response.json())['aruco']['markers']]==[4]
            e.stop('success')
            with gzip.open(e.directory/'events.jsonl.gz','rt') as f:events=[json.loads(line) for line in f]
            detection=next(x['data'] for x in events if x['event']=='aruco')
            assert detection['metadata']==dict(metadata,intrinsics=None) and detection['markers'][0]['id']==4
            image=next(x['data'] for x in events if x['event']=='image')
            assert (e.directory/image['path']).read_bytes()==jpeg.tobytes()
        finally:await client.close()
    asyncio.run(run())


def test_floor_preview_preserves_spacing_floor_height_and_common_motion():
    from ffw_teleop.human_demo_sim import floor_preview_positions
    poses=np.tile(np.eye(4),(2,2,1,1))
    poses[0,0,:3,3]=[-.35,.90,-.2]
    poses[0,1,:3,3]=[ .35,.95,-.1]
    poses[1,:,:3,3]=poses[0,:,:3,3]+[.1,-.1,-.2]
    basis=np.array([[0,0,-1],[-1,0,0],[0,1,0]])
    targets=floor_preview_positions(poses,basis,0)
    assert np.allclose(targets[0].mean(axis=0)[:2],[.5,0])
    assert np.allclose(targets[:,:,2],poses[:,:,1,3])
    assert np.allclose(targets[:,0]-targets[:,1],(poses[:,0,:3,3]-poses[:,1,:3,3])@basis.T)
    assert np.allclose(targets[1]-targets[0],[[.2,-.1,-.1]]*2)


def test_arm_fk_accounts_for_fixed_lift():
    import xml.etree.ElementTree as ET
    from ffw_teleop.human_demo_sim import Arm
    # Minimal independent prismatic + seven revolute chain.
    root=ET.Element('robot')
    lift=ET.SubElement(root,'joint',name='lift_joint',type='prismatic')
    ET.SubElement(lift,'parent',link='base');ET.SubElement(lift,'child',link='lift')
    ET.SubElement(lift,'axis',xyz='0 0 1')
    for i in range(1,8):
        j=ET.SubElement(root,'joint',name=f'arm_l_joint{i}',type='revolute')
        ET.SubElement(j,'parent',link='lift' if i==1 else f'arm_l_link{i-1}')
        ET.SubElement(j,'child',link=f'arm_l_link{i}')
        ET.SubElement(j,'axis',xyz='0 1 0');ET.SubElement(j,'limit',lower='-3.14',upper='3.14')
    q=np.zeros(7)
    original=Arm(root,'l').fk(q)
    lowered=Arm(root,'l',{'lift_joint':-.15}).fk(q)
    assert np.allclose(lowered[:3,3]-original[:3,3],[0,0,-.15])


def test_sdk_skeleton_and_manual_editor_api(tmp_path):
    from types import SimpleNamespace
    from ffw_teleop.human_demo import application,skeleton_event
    from aiohttp.test_utils import TestClient,TestServer
    async def run():
        e=HumanEpisodes(tmp_path)
        frame=SimpleNamespace(is_left=True,id='glove',profile_uuid='profile',timestamp_ms=100,tracking_state=0,
            positions=[SimpleNamespace(x=-i*.005,y=0.,z=0.) for i in range(25)],
            rotations=[SimpleNamespace(w=1.,x=0.,y=0.,z=0.) for _ in range(19)])
        skeleton_event(e,frame)
        client=TestClient(TestServer(application(e,'secret',require_camera=False)))
        await client.start_server()
        auth={'Authorization':'Bearer secret'}
        try:
            assert (await client.get('/mount-state')).status==401
            response=await client.get('/mount-assets/models/meta-quest-touch-pro-left.glb')
            assert response.status==200 and (await response.read())[:4]==b'glTF'
            response=await client.get('/mount-state',headers=auth)
            data=await response.json()
            assert data['skeletons']['left']['positions_m'][10]==[-.05,0.,0.]
            matrix=np.eye(4);matrix[0,3]=.02
            body=dict(action='mount-set',side='left',T_controller_wrist=matrix.tolist(),controller_model='meta-quest-touch-pro')
            response=await client.post('/control',headers=auth,json=body)
            assert response.status==200
            assert e.capture_calibration.mounts['left']['skeleton_profile_uuid']=='profile'
            e.start('recording')
            response=await client.post('/control',headers=auth,json=body)
            assert response.status==400
            e.stop()
            frame.timestamp_ms+=1;frame.tracking_state=2;skeleton_event(e,frame)
            assert 'skeleton_skipped_left' in e.latest
            response=await client.get('/mount-state',headers=auth)
            assert 'left' in (await response.json())['skeletons']
            frame.timestamp_ms+=1;frame.tracking_state=1;skeleton_event(e,frame)
            response=await client.get('/mount-state',headers=auth)
            assert 'left' not in (await response.json())['skeletons']
        finally:await client.close()
    asyncio.run(run())
