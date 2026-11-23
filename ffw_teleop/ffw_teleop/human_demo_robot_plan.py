"""Plan a short board-relative arm trial from a fresh robot/camera snapshot; no motor IO."""
import argparse
import json
from pathlib import Path
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
from .human_demo_robot import RobotFrames, load_live_robot_scene, table_relative_targets
from .human_demo_data import rigid
from .human_demo_retarget import hand_body_to_sdk_wrist
from .human_demo_scene import WORKSPACE


def observed_board(snapshot):
    observation = snapshot.get('board', {})
    if not observation.get('valid'):
        raise ValueError('A stable live head-camera board observation is required')
    age = (snapshot['ros_time_ns'] - observation['image_stamp_ns']) / 1e9
    if not -.1 <= age <= 1.:
        raise ValueError('Board observation is stale')
    if observation.get('reference_frame') != 'base_link':
        raise ValueError('Board must be expressed in base_link')
    return rigid(observation['T_base_board'])


def source_board_hands(data, indices, assumed_source_board=None):
    if 'T_board_wrist' in data:
        if not data['board_wrist_valid'][indices].all():
            raise ValueError('Invalid source board/wrist samples')
        return data['T_board_wrist'][indices], 'recorded calibrated board'
    if assumed_source_board is None:
        raise ValueError('Recording has no measured board frame; an explicit assumed source board is required')
    if 'wrist_target_poses' not in data:
        raise ValueError('Source has no corrected hand target poses')
    return np.linalg.inv(rigid(assumed_source_board)) @ data['wrist_target_poses'][indices], 'explicit assumed source board, not measured capture alignment'


def plan_trial(source, snapshot, output, seconds=.6, start=0., assumed_source_board=None, workspace=WORKSPACE):
    import mujoco
    if not np.isfinite([seconds,start]).all() or not 0 < seconds <= 30 or start < 0:
        raise ValueError('Trial must be a finite interval no longer than 30 seconds')
    snapshot = json.loads(Path(snapshot).read_text())
    board = observed_board(snapshot)
    data = np.load(source)
    if 'time_s' in data:
        times = data['time_s']
    else:
        times = (data['receive_monotonic_ns']-data['receive_monotonic_ns'][0])/1e9
    indices = np.flatnonzero((times >= start) & (times <= start+seconds))
    if len(indices) < 2:raise ValueError('Need at least two source samples')
    wrists, provenance = source_board_hands(data, indices, assumed_source_board)
    targets = table_relative_targets(wrists,board)
    body_targets = targets @ np.linalg.inv(hand_body_to_sdk_wrist(.057))
    config = json.loads(Path(workspace).read_text())
    # Scene placement follows the detected horizontal paper. Table dimensions
    # remain a conservative assumed rectangle until independently measured.
    config['simulation']['table_surface_height_m'] = float(board[2,3]-.0005)
    config['simulation']['table_centre_robot_m'] = [*board[:2,3], float(board[2,3]-.0005)]
    config['simulation']['board_centre_robot_m'] = board[:3,3].tolist()
    import tempfile
    with tempfile.TemporaryDirectory(prefix='board-trial-') as temp:
        scene_path = Path(temp)/'workspace.json';scene_path.write_text(json.dumps(config))
        m,d = load_live_robot_scene(snapshot['urdf'],scene_path)
    frames = RobotFrames(snapshot['urdf'])
    for name,value in snapshot['joints'].items():
        try:d.qpos[m.joint(name).qposadr[0]]=value
        except KeyError:pass
    base = d.qpos.copy();mujoco.mj_forward(m,d)
    nominal = frames.transforms(snapshot['joints'])
    for name in [f'arm_{s}_link{i}' for s in ('l','r') for i in range(1,8)] + ['left_hand_C_MC','right_hand_C_MC']:
        b=m.body(name).id
        if not np.allclose(d.xpos[b],nominal[name][:3,3],atol=1e-7) or not np.allclose(d.xmat[b].reshape(3,3),nominal[name][:3,:3],atol=1e-7):
            raise ValueError('Live robot and collision model FK disagree: '+name)
    names = [f'arm_{s}_joint{i}' for s in ('l','r') for i in range(1,8)]
    adr = np.array([m.joint(n).qposadr[0] for n in names]);rows=np.tile(base[adr],(len(indices),1));errors=np.zeros((len(indices),2,2))
    rng=np.random.default_rng(431)
    for si,side in enumerate(('left','right')):
        columns=np.arange(si*7,(si+1)*7);qadr=adr[columns];bid=m.body(side+'_hand_C_MC').id
        limits=[frames.root.find("joint[@name='%s']/limit"%names[k]) for k in columns]
        low=np.array([float(x.get('lower')) for x in limits]);high=np.array([float(x.get('upper')) for x in limits]);q=base[qadr].copy()
        for k in range(len(indices)):
            target=body_targets[k,si];old=q.copy()
            def residual(x):
                d.qpos[qadr]=x;mujoco.mj_kinematics(m,d)
                return np.r_[d.xpos[bid]-target[:3,3], .12*Rotation.from_matrix(target[:3,:3]@d.xmat[bid].reshape(3,3).T).as_rotvec(), .008*(x-old)]
            seeds=[q]
            if k==0:seeds += [np.clip(q+rng.normal(0,1.2,7),low+1e-6,high-1e-6) for _ in range(24)]
            fits=[least_squares(residual,np.clip(seed,low+1e-6,high-1e-6),bounds=(low,high),max_nfev=200 if k==0 else 100) for seed in seeds]
            q=min(fits,key=lambda fit:np.linalg.norm(residual(fit.x))).x;residual(q);rows[k,columns]=q
            errors[k,si]=[np.linalg.norm(d.xpos[bid]-target[:3,3]),Rotation.from_matrix(target[:3,:3]@d.xmat[bid].reshape(3,3).T).magnitude()]
    if errors[:,:,0].max()>.015 or errors[:,:,1].max()>np.deg2rad(10):
        raise ValueError('Unreachable target: max position %.3f m; angle %.1f deg'%(errors[:,:,0].max(),np.rad2deg(errors[:,:,1].max())))
    # Fixed lift and held fingers: build slow joint-space approach, never an
    # online Cartesian inverse near singularities.
    initial=base[adr];duration=max(20.,float(np.max(np.abs(rows[0]-initial)))*1.875/.10);points=[]
    for u in np.linspace(0,1,int(duration*20)+1):
        blend=10*u**3-15*u**4+6*u**5;points.append([float(.5+u*duration),((1-blend)*initial+blend*rows[0]).tolist()])
    for k in range(1,len(indices)):
        dt=max(float(times[indices[k]]-times[indices[k-1]])/.20,float(np.max(np.abs(rows[k]-rows[k-1])))/.10)
        points.append([points[-1][0]+dt,rows[k].tolist()])
    points.append([points[-1][0]+1.,rows[-1].tolist()])
    mujoco.mj_forward(m,d)
    d.qpos[:]=base;mujoco.mj_forward(m,d)
    baseline={tuple(sorted((c.geom1,c.geom2))):float(c.dist) for c in d.contact}
    def category(g):
        n=m.body(m.geom_bodyid[g]).name
        if n=='table':return 'table'
        if n=='world' or n.startswith(('cube','human_demo','aruco','pan','spatula','target')):return 'scene'
        return 'robot'
    table_id=m.geom('table_top').id
    moving_geoms=[g for g in range(m.ngeom) if m.geom_contype[g] and m.body(m.geom_bodyid[g]).name.startswith(('arm_l_','arm_r_','left_','right_')) and 'wheel' not in m.body(m.geom_bodyid[g]).name]
    minimum_table_clearance=.10
    previous=initial;checks=0
    for t,row in points:
        for u in np.linspace(0,1,10):
            d.qpos[:]=base;d.qpos[adr]=(1-u)*previous+u*np.asarray(row);mujoco.mj_forward(m,d);checks+=1
            for g in moving_geoms:
                distance=mujoco.mj_geomDistance(m,d,g,table_id,.10,None)
                minimum_table_clearance=min(minimum_table_clearance,float(distance))
                if distance<.02:raise ValueError('Less than 20 mm table clearance at %.2fs: %s'%(t,m.body(m.geom_bodyid[g]).name))
            for c in d.contact:
                types={category(c.geom1),category(c.geom2)}
                is_table=types=={'robot','table'}
                is_self=types=={'robot'}
                threshold=-.001 if is_table else min(-.001,baseline.get(tuple(sorted((c.geom1,c.geom2))),0.)-.001)
                if (is_table or is_self) and c.dist<threshold:
                    raise ValueError('Collision on approach/replay at %.2fs: %s / %s'%(t,m.body(m.geom_bodyid[c.geom1]).name,m.body(m.geom_bodyid[c.geom2]).name))
        previous=np.asarray(row)
    result=dict(joint_names=names,points=points,expected_joints=snapshot['joints'],urdf=snapshot['urdf'],
                lift_fixed_m=snapshot['joints']['lift_joint'],approach_s=duration,
                source_duration_s=float(times[indices[-1]]-times[indices[0]]),arm_speed_limit_rad_s=.10,
                board_anchor=snapshot['board'],source_board_provenance=provenance,
                position_error_max_m=errors[:,:,0].max(axis=0).tolist(),rotation_error_max_deg=np.rad2deg(errors[:,:,1].max(axis=0)).tolist(),
                minimum_table_clearance_m=minimum_table_clearance,target_wrist_poses=targets.tolist(),board_relative_wrists=np.asarray(wrists).tolist(),collision_samples=checks,
                warning='Nominal camera URDF and approximate collision meshes; table size is assumed. No motor IO.')
    Path(output).write_text(json.dumps(result,ensure_ascii=False,indent=2))
    print(json.dumps({k:v for k,v in result.items() if k in ('source_duration_s','approach_s','position_error_max_m','rotation_error_max_deg','collision_samples','source_board_provenance')},indent=2))


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('source',type=Path);p.add_argument('--snapshot',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--seconds',type=float,default=.6);p.add_argument('--start',type=float,default=0)
    p.add_argument('--assumed-source-board',type=Path,help='Explicit workspace JSON, only for uncalibrated source preview poses')
    a=p.parse_args();assumed=None
    if a.assumed_source_board:assumed=json.loads(a.assumed_source_board.read_text())['simulation']['T_robot_board']
    plan_trial(a.source,a.snapshot,a.output,a.seconds,a.start,assumed)


if __name__=='__main__':main()
