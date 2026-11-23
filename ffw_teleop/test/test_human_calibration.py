import json
import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from ffw_teleop.human_demo_calibration import Calibration, fit_mount,fit_board
from ffw_teleop.quest_camera_calibration import parse_camera2_dump,for_frame


def transform(rotation=(0,0,0),position=(0,0,0)):
    t=np.eye(4);t[:3,:3]=Rotation.from_rotvec(rotation).as_matrix();t[:3,3]=position
    return t


def test_mount_recovers_position_rotation_and_rejects_missing_pairs():
    rng=np.random.default_rng(32)
    mount=transform([.2,-.1,.3],[.03,-.04,.08])
    pairs=[]
    for _ in range(60):
        c=transform(rng.normal(0,.3,3),rng.normal(0,.2,3))
        pairs.append((c,c@mount))
    # Some optical outliers must not bias the mount.
    for _ in range(10):
        c=transform();pairs.append((c,transform(position=(.2,.2,.2))))
    result=fit_mount(pairs)
    assert np.allclose(result['T_controller_wrist'],mount,atol=1e-8)
    assert result['rejected_samples']==10
    with pytest.raises(ValueError,match='不足'):fit_mount(pairs[:5])


def test_hand_eye_recovers_head_camera_and_board_and_rejects_one_axis():
    rng=np.random.default_rng(41)
    camera=transform([3.,.02,-.05],[.03,-.02,-.07])
    board=transform([1.5,.1,-.3],[.5,.7,-1.])
    samples=[]
    for _ in range(50):
        head=transform(rng.normal(0,.25,3),rng.normal(0,.1,3))
        samples.append((head,np.linalg.inv(camera)@np.linalg.inv(head)@board))
    fitted=fit_board(samples)
    assert np.allclose(fitted['T_head_camera'],camera,atol=1e-7)
    assert np.allclose(fitted['T_world_board'],board,atol=1e-7)
    with pytest.raises(ValueError,match='不足'):fit_board(samples[:10])
    one_axis=[]
    for yaw in np.linspace(-.4,.4,40):
        head=transform([0,yaw,0])
        one_axis.append((head,np.linalg.inv(camera)@np.linalg.inv(head)@board))
    with pytest.raises(ValueError,match='回転方向'):fit_board(one_axis)


def test_hand_sources_never_substitute_for_controller_and_board_resets(tmp_path):
    c=Calibration(tmp_path/'calibration.json');c.reset_reference('one')
    c.board=dict(T_world_board=transform(position=[1,2,3]).tolist())
    c.mounts={'left':dict(T_controller_wrist=transform(position=[.1,0,0]).tolist())}
    controller=transform(position=[1.5,2,3])
    wrist=controller@transform(position=[.1,0,0])
    data=dict(reference_id='one',client_ms=10,left=controller.flatten(order='F').tolist(),
        hands={'left':dict(wrist=wrist.flatten(order='F').tolist(),emulated_position=False)})
    derived=c.observe_xr(data)
    assert np.allclose(np.array(derived['T_board_left_controller'])[:3,3],[.5,0,0])
    assert np.allclose(np.array(derived['T_board_left_wrist'])[:3,3],[.6,0,0])
    assert c.status()['mount_samples']['left']==1
    data['left']=None
    assert c.observe_xr(data)=={}
    assert c.status()['mount_samples']['left']==1
    c.reset_reference('two')
    assert c.board is None and not c.xr and c.status()['mount_samples']['left']==0


def test_parse_device_intrinsics_and_refuse_cropped_or_wrong_camera():
    dump='''Device 2 maps to "51"
Camera HAL device device@1.1/virtual/51 (v3.1) static information:
 android.lens.intrinsicCalibration (8000a): float[5]
 [1200 1200 864 864 0 ]
 android.sensor.info.preCorrectionActiveArraySize (f000a): int32[4]
 [0 0 1728 1728 ]
'''
    configs=parse_camera2_dump(dump,'test-device')
    m=dict(camera_label='camera 2, facing back',width=1280,height=1280,settings=dict(resizeMode='none'))
    cfg=for_frame(configs,m)
    assert np.isclose(cfg['K'][0][2],640)
    assert cfg['serial']=='test-device'
    assert for_frame(configs,dict(m,height=960)) is None
    assert for_frame(configs,dict(m,camera_label='avatar')) is None
    assert for_frame(configs,dict(m,settings=dict(resizeMode='crop-and-scale'))) is None


def test_board_pose_recovers_metric_position_from_known_projection():
    cv2=pytest.importorskip('cv2')
    from ffw_teleop.human_demo_aruco import ArucoDetector
    k=np.array([[800,0,640],[0,800,640],[0,0,1.]])
    board=transform([np.pi,0,0],[.02,-.03,.6])
    corners=np.array([[-.025,.025,0],[.025,.025,0],[.025,-.025,0],[-.025,-.025,0]])
    markers=[];centres={}
    rvec,_=cv2.Rodrigues(board[:3,:3])
    for i in range(9):
        centre=np.array([(i%3-1)*.06,(1-i//3)*.07,0])
        centres[str(i)]=centre.tolist()
        pixels,_=cv2.projectPoints(corners+centre,rvec,board[:3,3],k,np.zeros(5))
        markers.append(dict(id=i,corners=pixels.reshape(4,2).tolist()))
    result=ArucoDetector().board_pose(dict(markers=markers),dict(K=k.tolist(),distortion=[0.]*5,source='synthetic'),dict(dictionary='DICT_4X4_50',marker_side_m=.05,marker_centres_board_m=centres))
    assert result is not None
    assert np.allclose(result['T_camera_board'],board,atol=1e-6)
    assert result['reprojection_px']<1e-5


def test_calibration_is_saved_with_episode_and_board_wrist_export(tmp_path):
    from ffw_teleop.human_demo_data import HumanEpisodes
    from ffw_teleop.export_human_demo import export_episode
    e=HumanEpisodes(tmp_path)
    c=Calibration();e.capture_calibration=c
    e.record('reference',dict(reference_id='one'))
    c.board=dict(T_world_board=transform(position=[1,2,3]).tolist())
    c.mounts={s:dict(T_controller_wrist=transform(position=[.1,0,0]).tolist()) for s in ('left','right')}
    e.start('calibrated')
    for s in ('left','right'):e.record('glove_'+s,dict(source_timestamp_ms=123,angles_degrees=[0.]*20))
    controller=transform(position=[1.5,2,3]).flatten(order='F').tolist()
    e.record('xr',dict(reference_id='one',head=np.eye(4).flatten(order='F').tolist(),left=controller,right=controller))
    e.stop('success')
    a=np.load(export_episode(e.directory))
    assert a['board_wrist_valid'].all()
    assert np.allclose(a['T_board_wrist'][0,0,:3,3],[.6,0,0])
    manifest=json.loads((e.directory/'manifest.json').read_text())
    assert manifest['capture_calibration']['mounts']['left']['T_controller_wrist'][0][3]==.1


def test_post_recording_calibration_matches_reference_and_never_invents_wrist(tmp_path):
    from ffw_teleop.human_demo_data import HumanEpisodes
    from ffw_teleop.export_human_demo import export_episode
    e=HumanEpisodes(tmp_path/'episodes');e.start('test')
    matrix=transform(position=[1.5,2,3]).flatten(order='F').tolist()
    e.record('xr',dict(reference_id='one',head=np.eye(4).flatten(order='F').tolist(),left=matrix,right=matrix))
    e.stop('success')
    capture=dict(mounts={},board=dict(reference_id='one',T_world_board=transform(position=[1,2,3]).tolist()))
    file=tmp_path/'calibration.json';file.write_text(json.dumps(capture))
    a=np.load(export_episode(e.directory,calibration_file=file))
    assert a['board_controller_valid'].all()
    assert not a['board_wrist_valid'].any()
    assert np.allclose(a['T_board_controller'][0,0,:3,3],[.5,0,0])
    capture['board']['reference_id']='two';file.write_text(json.dumps(capture))
    with pytest.raises(ValueError,match='reference_id'):
        export_episode(e.directory,calibration_file=file)


def test_manual_mount_persists_without_reusing_old_world_reference(tmp_path):
    file=tmp_path/'calibration.json';c=Calibration(file)
    m=transform([.1,.2,.3],[.02,-.04,.08])
    saved=c.set_manual_mount('left',m,'meta-quest-touch-pro','profile')
    assert saved['target_frame']=='contactglove_skeleton_root'
    reloaded=Calibration(file)
    assert np.allclose(reloaded.mounts['left']['T_controller_wrist'],m)
    assert reloaded.board is None
    with pytest.raises(ValueError,match='side'):c.set_manual_mount('other',m,'meta-quest-touch-pro')
    with pytest.raises(ValueError):c.set_manual_mount('left',np.diag([2,1,1,1]),'meta-quest-touch-pro')
    with pytest.raises(ValueError,match='50cm'):c.set_manual_mount('left',transform(position=[1,0,0]),'meta-quest-touch-pro')


def test_saved_camera_extrinsic_reanchors_only_the_same_device(tmp_path):
    file=tmp_path/'calibration.json'
    camera=transform([3.,.03,.05],[.03,-.02,-.07])
    extrinsic=dict(T_head_camera=camera.tolist(),camera=['camera 2, facing back',1280,1280,'device-one'])
    file.write_text(json.dumps(dict(mounts={},camera_extrinsic=extrinsic)))
    c=Calibration(file);c.reset_reference('new')
    board=transform([1.5,.1,-.3],[.5,.7,-1.])
    metadata=dict(reference_id='new',client_ms=100,camera_label='camera 2, facing back',intrinsics_source=dict(serial='wrong-device'))
    h=transform();c.observe_xr(dict(reference_id='new',client_ms=100,head=h.flatten(order='F').tolist()))
    detection=dict(width=1280,height=1280,pose=dict(T_camera_board=(np.linalg.inv(camera)@board).tolist(),marker_ids=list(range(9)),reprojection_px=.3))
    for _ in range(25):c.observe_board(detection,metadata)
    assert c.board is None
    metadata['intrinsics_source']['serial']='device-one'
    for _ in range(20):c.observe_board(detection,metadata)
    assert np.allclose(c.board['T_world_board'],board)
    assert c.board['reference_id']=='new'


def test_sdk_hand_axes_and_wrist_face_are_matched_without_start_pose_reset():
    from ffw_teleop.human_demo_retarget import hand_body_to_sdk_wrist,board_hand_targets
    reference=hand_body_to_sdk_wrist(.057)
    # SDK finger direction is -X, robot hand finger direction is +Z.
    assert np.allclose(reference[:3,:3]@[-1,0,0],[0,0,1])
    # SDK +Y is the back of the hand, opposite robot +X palm normal.
    assert np.allclose(reference[:3,:3]@[0,1,0],[-1,0,0])
    robot_board=transform([0,0,-np.pi/2],[.62,0,.7005])
    board_hands=np.tile(np.eye(4),(2,2,1,1))
    board_hands[0,0,:3,3]=[-.2,0,.1]
    board_hands[0,1,:3,3]=[ .2,0,.1]
    board_hands[1,:,:3,3]=board_hands[0,:,:3,3]+[0,.1,0]
    wrists,bodies=board_hand_targets(board_hands,robot_board,.057)
    assert np.allclose(wrists[0,0,:3,3],[.62,.2,.8005])
    assert np.allclose(wrists[0,1,:3,3],[.62,-.2,.8005])
    assert np.allclose(wrists[1,:,:3,3]-wrists[0,:,:3,3],[[.1,0,0]]*2)
    # Offset rotation is applied about the wrist, and the mechanical wrist face
    # still lands at the measured SDK root after constructing body targets.
    assert np.allclose(bodies@reference,wrists)
