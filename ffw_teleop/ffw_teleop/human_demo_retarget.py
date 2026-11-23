"""Retarget board-referenced, manually aligned SDK hands using the actual MuJoCo model."""
import argparse
import json
from pathlib import Path
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
from .contactglove_mapping import FLEXION,ABDUCTION
from .human_demo_data import rigid
from .human_demo_scene import WORKSPACE


def hand_body_to_sdk_wrist(wrist_face_offset):
    """Mechanical convention: hand +Z=fingers, +X=palm; SDK +X=toward wrist,+Y=back.

    Proxy wrist origin is the model's wrist mounting face, not a claimed anatomical measurement.
    """
    t=np.eye(4)
    t[:3,:3]=[[0,-1,0],[0,0,1],[-1,0,0]]
    t[:3,3]=[0,0,-wrist_face_offset]
    return rigid(t)


def board_hand_targets(board_wrists, robot_board, wrist_face_offset):
    wrists=np.asarray(board_wrists,dtype=float)
    base=rigid(robot_board)@wrists
    return base,base@np.linalg.inv(hand_body_to_sdk_wrist(wrist_face_offset))


def convert(arrays,output,calibration,workspace=WORKSPACE,hz=10.,lift=-.15,alignment='board',height_offset=0.):
    import mujoco
    from ffw_sharpa_sim.model import HAND_WRIST_OFFSET
    from .human_demo_scene import load_scene
    capture=json.loads(Path(calibration).read_text())
    for side in ('left','right'):
        mount=capture.get('mounts',{}).get(side,{})
        if mount.get('target_frame')!='contactglove_skeleton_root':
            raise ValueError(side+'のSDK手モデル取り付け補正が必要です')
        rigid(mount['T_controller_wrist'])
    if alignment == 'board' and not capture.get('board'):
        raise ValueError('記録と同じreference_idのArUco校正が必要です')
    board_ref=capture.get('board',{}).get('reference_id') if capture.get('board') else None
    config=json.loads(Path(workspace).read_text())
    robot_board=rigid(config['simulation']['T_robot_board'])
    a=np.load(arrays)
    if alignment == 'board' and not np.all(a['reference_id']==board_ref):
        raise ValueError('記録とArUco校正のreference_idが一致しません')
    if alignment == 'board':
        valid=a['board_wrist_valid'].all(axis=1)&a['glove_valid'].all(axis=1)
    else:
        if len(np.unique(a['reference_id'])) != 1:raise ValueError('reference resetを含みます')
        valid=a['pose_valid'].all(axis=1)&a['head_valid'].all(axis=1)&a['glove_valid'].all(axis=1)
    times=(a['receive_monotonic_ns']-a['receive_monotonic_ns'][0])/1e9
    indices=[];last=-np.inf
    for i,t in enumerate(times):
        if valid[i] and t-last>=1/hz:indices.append(i);last=t
    if not indices:raise ValueError('校正済みの両手とグローブの有効サンプルがありません')
    indices=np.array(indices)
    if alignment == 'board':
        wrist_targets,body_targets=board_hand_targets(a['T_board_wrist'][indices],robot_board,HAND_WRIST_OFFSET)
    else:
        first=indices[0]
        head=a['T_world_controller'][first,0]@np.linalg.inv(a['T_head_controller'][first,0])
        forward=-head[:3,2].copy();forward[1]=0
        if np.linalg.norm(forward)<1e-6:raise ValueError('initial head yaw is undefined')
        forward/=np.linalg.norm(forward)
        up=np.array([0.,1.,0.]);basis=np.stack([forward,np.cross(up,forward),up])
        world_wrists=a['T_world_controller'][indices]@np.array([capture['mounts'][side]['T_controller_wrist'] for side in ('left','right')])
        robot_world=np.eye(4);robot_world[:3,:3]=basis
        midpoint=(basis@world_wrists[0,:,:3,3].T).mean(axis=1)
        robot_world[:3,3]=[.35-midpoint[0],-midpoint[1],0.]
        wrist_targets,body_targets=board_hand_targets(world_wrists,robot_world,HAND_WRIST_OFFSET)
    if not np.isfinite(height_offset):raise ValueError('height offset must be finite')
    wrist_targets[:,:,2,3]+=height_offset
    body_targets[:,:,2,3]+=height_offset
    m,d=load_scene(workspace)
    lift_id=m.joint('lift_joint').id
    if not m.jnt_range[lift_id,0]<=lift<=m.jnt_range[lift_id,1]:raise ValueError('lift is outside model limits')
    d.qpos[m.joint('lift_joint').qposadr[0]]=lift
    controlled=[m.joint(i).name for i in range(m.njnt) if m.joint(i).name.startswith(('arm_l_joint','arm_r_joint','left_','right_')) or m.joint(i).name=='lift_joint']
    addresses=[m.joint(name).qposadr[0] for name in controlled]
    result=np.tile(d.qpos[addresses],(len(indices),1))
    errors=np.zeros((len(indices),2,2));actual_wrists=np.zeros((len(indices),2,3))
    cfg=capture.get('glove_config',{})
    clamps=[0,0]
    wrist_offset=hand_body_to_sdk_wrist(HAND_WRIST_OFFSET)
    for side_index,side in enumerate(('left','right')):
        short='l' if side=='left' else 'r'
        arm_names=[f'arm_{short}_joint{i}' for i in range(1,8)]
        jids=[m.joint(name).id for name in arm_names];qadr=m.jnt_qposadr[jids]
        low=m.jnt_range[jids,0];high=m.jnt_range[jids,1]
        bid=m.body(side+'_hand_C_MC').id
        q=d.qpos[qadr].copy()
        for k,i in enumerate(indices):
            target=body_targets[k,side_index];old=q.copy()
            def residual(x):
                d.qpos[qadr]=x;mujoco.mj_kinematics(m,d)
                position=d.xpos[bid]-target[:3,3]
                angle=Rotation.from_matrix(target[:3,:3]@d.xmat[bid].reshape(3,3).T).as_rotvec()
                return np.r_[position,.12*angle,.003*(x-old)]
            seeds=[q]
            if k==0:
                relaxed=np.zeros(7);relaxed[4]=-np.pi/2 if side=='left' else np.pi/2
                seeds.append(relaxed)
                alternate=q.copy();alternate[2]=.8 if side=='left' else -.8;seeds.append(alternate)
            fits=[least_squares(residual,np.clip(seed,low+1e-8,high-1e-8),bounds=(low,high),max_nfev=180 if k==0 else 60,ftol=1e-5) for seed in seeds]
            q=min(fits,key=lambda fit:np.linalg.norm(residual(fit.x))).x
            residual(q)
            actual=np.eye(4);actual[:3,:3]=d.xmat[bid].reshape(3,3);actual[:3,3]=d.xpos[bid]
            actual_wrist=actual@wrist_offset
            actual_wrists[k,side_index]=actual_wrist[:3,3]
            errors[k,side_index]=[np.linalg.norm(actual_wrist[:3,3]-wrist_targets[k,side_index,:3,3]),Rotation.from_matrix(target[:3,:3]@actual[:3,:3].T).magnitude()]
            for name,value in zip(arm_names,q):result[k,controlled.index(name)]=value
            angles=a['glove_angles_degrees'][i,side_index]
            for mapping,prefix in ((FLEXION,''),(ABDUCTION,'abduction_')):
                for n,(joint,index,_) in enumerate(mapping):
                    name=f'{side}_{joint}'
                    if name not in controlled:continue
                    value=np.deg2rad(angles[index])*cfg.get(f'{side}_{prefix}gains',[1.]*len(mapping))[n]+cfg.get(f'{side}_{prefix}offsets',[0.]*len(mapping))[n]
                    lo,hi=m.jnt_range[m.joint(name).id];clamps[side_index]+=int(value<lo or value>hi)
                    result[k,controlled.index(name)]=np.clip(value,lo,hi)
        print(f'{side}: {len(indices)} calibrated frames solved',flush=True)
    output=Path(output)
    np.savez_compressed(output,time_s=times[indices]-times[indices[0]],joint_names=np.array(controlled),positions=result,
        ik_errors=errors,wrist_target_positions_m=wrist_targets[:,:,:3,3],
        wrist_target_poses=wrist_targets,wrist_actual_positions_m=actual_wrists,
        robot_board=robot_board if alignment == 'board' else np.empty((0,4,4)),source_indices=indices)
    report=dict(source=str(Path(arrays).resolve()),calibration=str(Path(calibration).resolve()),
        capture_calibration=capture,frames=len(indices),duration_s=float(times[indices[-1]]-times[indices[0]]),
        alignment=('Measured recording-reference ArUco board; saved manual controller-to-SDK-hand transform; no first-pose re-anchoring or hand-spacing rescale.' if alignment == 'board' else 'Saved manual mounts; WebXR floor height and first-head yaw; shared horizontal translation places initial wrist midpoint at x=0.35m/y=0. No measured ArUco alignment.'),
        robot_board=robot_board.tolist() if alignment == 'board' else None,lift_joint_m=lift,
        robot_hand_reference='Model wrist mounting face; hand +Z fingers/+X palm mapped to SDK -X fingers/+Y back. This geometric convention is not an anatomical fit.',
        model_wrist_face_offset_m=HAND_WRIST_OFFSET,hand_height_offset_m=height_offset,
        position_error_p50_m=np.median(errors[:,:,0],axis=0).tolist(),position_error_p95_m=np.percentile(errors[:,:,0],95,axis=0).tolist(),
        rotation_error_p95_deg=np.rad2deg(np.percentile(errors[:,:,1],95,axis=0)).tolist(),
        reachable_fraction_5cm_20deg=((errors[:,:,0]<.05)&(errors[:,:,1]<np.deg2rad(20))).mean(axis=0).tolist(),
        finger_clamped_values=clamps,side_order=['left','right'],
        mode='MuJoCo kinematic preview, not collision-avoiding motion or hardware execution')
    output.with_suffix('.json').write_text(json.dumps(report,ensure_ascii=False,indent=2))
    print(json.dumps({k:v for k,v in report.items() if k!='capture_calibration'},ensure_ascii=False,indent=2))


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('arrays',type=Path);p.add_argument('--calibration',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);p.add_argument('--workspace',type=Path,default=WORKSPACE)
    p.add_argument('--alignment',choices=('board','floor'),default='board')
    p.add_argument('--hz',type=float,default=10.);p.add_argument('--lift',type=float,default=-.15)
    p.add_argument('--height-offset',type=float,default=0.,help='共通の手目標高さ補正 [m]、負で下げる')
    a=p.parse_args()
    if not np.isfinite(a.hz) or a.hz<=0:p.error('hz must be positive')
    convert(a.arrays,a.output,a.calibration,a.workspace,a.hz,a.lift,a.alignment,a.height_offset)


if __name__=='__main__':main()
