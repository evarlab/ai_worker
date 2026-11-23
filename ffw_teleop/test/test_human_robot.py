import numpy as np
import pytest
from ffw_teleop.human_demo_robot import RobotFrames, table_relative_targets

XML='''<robot name="test"><link name="base_link"/><link name="torso"/><link name="camera"/><joint name="lift" type="prismatic"><parent link="base_link"/><child link="torso"/><origin xyz="0 0 1"/><axis xyz="0 0 1"/></joint><joint name="camera_fix" type="fixed"><parent link="torso"/><child link="camera"/><origin xyz="0.1 0 0.2" rpy="0 0 1.5707963267948966"/></joint></robot>'''

def test_camera_fk_uses_live_lift_without_sim_root_offset():
    frames=RobotFrames(XML)
    high=frames.transforms({'lift':0})['camera']
    low=frames.transforms({'lift':-.2})['camera']
    np.testing.assert_allclose(high[:3,3],[.1,0,1.2])
    np.testing.assert_allclose(low[:3,3],[.1,0,1.])
    np.testing.assert_allclose(low[:3,:3]@[1,0,0],[0,1,0],atol=1e-12)
    with pytest.raises(ValueError,match='Missing'):frames.transforms({})
    with pytest.raises(ValueError,match='not fixed'):frames.fixed_transform('base_link','camera')

def test_shared_board_transform_preserves_table_height_and_between_hands():
    hands=np.tile(np.eye(4),(3,2,1,1));hands[:,0,:3,3]=[.1,.2,.06];hands[:,1,:3,3]=[-.2,.1,.10]
    board=np.eye(4);board[:3,:3]=[[0,1,0],[-1,0,0],[0,0,1]];board[:3,3]=[.62,0,.7]
    result=table_relative_targets(hands,board)
    np.testing.assert_allclose(result[:,:,2,3],[[.76,.8]]*3)
    np.testing.assert_allclose(np.linalg.inv(result[:,0])@result[:,1],np.linalg.inv(hands[:,0])@hands[:,1],atol=1e-12)
    # A different fixed lift changes arm IK, not the board-referenced target.
    np.testing.assert_allclose(table_relative_targets(hands,board),result)


def test_board_stability_rejects_wrong_height_and_motion():
    from ffw_teleop.robot_board import stable_board
    board=np.eye(4);board[:3,3]=[.62,0,.7005]
    samples=[board.copy() for _ in range(20)]
    result=stable_board(samples)
    np.testing.assert_allclose(result['T_base_board'],board)
    samples[-1][0,3]+=.04
    with pytest.raises(ValueError,match='unstable'):stable_board(samples)
    with pytest.raises(ValueError,match='Waiting'):stable_board(samples[:10])
    board[2,3]=.9
    with pytest.raises(ValueError,match='height'):stable_board([board]*20)
    board[2,3]=.7005;board[:3,:3]=np.diag([1,-1,-1])
    with pytest.raises(ValueError,match='face-up'):stable_board([board]*20)


def test_rectified_camera_requires_matching_optical_frame_and_dimensions():
    from types import SimpleNamespace
    from ffw_teleop.robot_board import rectified_calibration
    info=SimpleNamespace(width=640,height=480,header=SimpleNamespace(frame_id='optical'),p=[400,0,320,0,0,400,240,0,0,0,1,0])
    result=rectified_calibration(640,480,'optical',info)
    assert result['K'][0][0]==400 and result['distortion']==[0]*5
    with pytest.raises(ValueError):rectified_calibration(640,480,'camera_link',info)
    with pytest.raises(ValueError):rectified_calibration(320,240,'optical',info)
    info.p[3]=-.1
    with pytest.raises(ValueError,match='left-camera'):rectified_calibration(640,480,'optical',info)


def test_observation_freshness_uses_robot_clock_and_rejects_invalid():
    from ffw_teleop.human_demo_robot_plan import observed_board
    snapshot=dict(ros_time_ns=5_000_000_000,board=dict(valid=True,image_stamp_ns=4_800_000_000,reference_frame='base_link',T_base_board=np.eye(4).tolist()))
    np.testing.assert_allclose(observed_board(snapshot),np.eye(4))
    snapshot['ros_time_ns']=7_000_000_000
    with pytest.raises(ValueError,match='stale'):observed_board(snapshot)
    snapshot['board']['valid']=False
    with pytest.raises(ValueError,match='stable'):observed_board(snapshot)


def test_uncalibrated_source_requires_explicit_assumption_and_keeps_pair_transform():
    from ffw_teleop.human_demo_robot_plan import source_board_hands
    wrists=np.tile(np.eye(4),(2,2,1,1));wrists[:,0,:3,3]=[.5,.2,.76];wrists[:,1,:3,3]=[.6,-.2,.8]
    data=dict(wrist_target_poses=wrists);board=np.eye(4);board[:3,3]=[.62,0,.7005]
    with pytest.raises(ValueError,match='explicit'):source_board_hands(data,[0,1])
    relative,provenance=source_board_hands(data,[0,1],board)
    np.testing.assert_allclose(relative[:,:,2,3],[[.0595,.0995]]*2)
    assert 'not measured' in provenance
    destination=np.eye(4);destination[:3,3]=[.75,.1,.7005]
    result=table_relative_targets(relative,destination)
    np.testing.assert_allclose(result[:,:,2,3],wrists[:,:,2,3])
    np.testing.assert_allclose(np.linalg.inv(result[:,0])@result[:,1],np.linalg.inv(wrists[:,0])@wrists[:,1],atol=1e-12)


def test_recorded_fingers_all_moves_thumb_and_four_fingers():
    from ffw_teleop.human_demo_robot import recorded_finger_targets
    cfg=dict(controlled_fingers='all',include_abduction=True,left_gains=[1.]*15,left_offsets=[0.]*15,left_lower=[0.]*15,left_upper=[1.2]*15,left_abduction_gains=[1.]*5,left_abduction_offsets=[0.]*5)
    current={'left_thumb_CMC_FE':0.,'left_index_PIP':0.,'left_pinky_DIP':0.,'left_thumb_CMC_AA':.1}
    from ffw_teleop.contactglove_mapping import FLEXION,ABDUCTION
    for name,_,_ in FLEXION+ABDUCTION:current.setdefault('left_'+name,0.)
    limits={name:(-1.,1.2) for name in current}
    result=recorded_finger_targets([30.]*20,cfg,'left',current,limits)
    assert result['left_thumb_CMC_FE']>.5 and result['left_index_PIP']>.5 and result['left_pinky_DIP']>.5
    assert result['left_thumb_CMC_AA']==.1
    cfg['controlled_fingers']='four_fingers'
    result=recorded_finger_targets([30.]*20,cfg,'left',current,limits)
    assert result['left_thumb_CMC_FE']==0 and result['left_index_PIP']>.5
    cfg['controlled_fingers']='unknown'
    with pytest.raises(ValueError):recorded_finger_targets([30.]*20,cfg,'left',current,limits)


def test_fixed_board_tracking_preserves_anchor_and_never_refreshes_cached_timestamp():
    from ffw_teleop.human_demo_robot import track_fixed_board
    anchor=np.eye(4);anchor[2,3]=.7005
    sample=dict(valid=True,T_base_board=anchor.tolist(),image_stamp_ns=1_000_000_000)
    verified=track_fixed_board(sample,anchor,1_100_000_000)
    missing=dict(valid=False,reason='No joint feedback within 100 ms of image')
    cached=track_fixed_board(missing,anchor,1_400_000_000,verified)
    assert cached['image_stamp_ns']==1_000_000_000
    with pytest.raises(ValueError,match='stale'):track_fixed_board(missing,anchor,1_600_000_000,cached)
    candidate=dict(valid=False,reason='Waiting for stable board samples (1/20)',T_base_board_candidate=anchor.tolist(),image_stamp_ns=1_600_000_000)
    refreshed=track_fixed_board(candidate,anchor,1_700_000_000,cached)
    assert refreshed['image_stamp_ns']==1_600_000_000
    moved=anchor.copy();moved[0,3]+=.02;candidate['T_base_board_candidate']=moved.tolist()
    with pytest.raises(ValueError,match='moved'):track_fixed_board(candidate,anchor,1_700_000_000,cached)
    with pytest.raises(ValueError,match='stopped'):track_fixed_board(dict(valid=False,reason='Observer stopped'),anchor,1_100_000_000,verified)
