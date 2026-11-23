"""Preview exported human joint trajectories using the existing AI Worker MuJoCo model.

Kinematic replay via mj_forward: no inferred object interaction or trained policy.
"""
import argparse
import json
from pathlib import Path
import time
import numpy as np
import mujoco
from .human_demo_scene import load_scene, WORKSPACE


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('trajectory',type=Path,nargs='?')
    p.add_argument('--scene-only',action='store_true')
    p.add_argument('--workspace',type=Path,default=WORKSPACE)
    p.add_argument('--loop',action='store_true')
    p.add_argument('--show-targets',action='store_true',help='目標の手モデル原点（未補正データならコントローラ位置）を球で表示')
    p.add_argument('--check',action='store_true',help='headless: validate every frame and save report')
    args=p.parse_args()
    model,data=load_scene(args.workspace)
    if args.scene_only:
        if args.check:
            top=model.geom('table_top')
            assert np.isclose(data.geom_xpos[top.id,2]+top.size[2],.7)
            assert all(model.body('aruco_id_'+str(i)).id >= 0 for i in range(9))
            print('PASS: table surface 700 mm; DICT_4X4_50 IDs 0-8; assumed board at x=0.62 m.')
            return
        from mujoco import viewer as mj_viewer
        with mj_viewer.launch_passive(model,data) as viewer:
            viewer.cam.lookat[:]=[.45,0,.7];viewer.cam.distance=1.8;viewer.cam.azimuth=145;viewer.cam.elevation=-35
            while viewer.is_running():
                viewer.sync();time.sleep(.02)
        return
    if args.trajectory is None:
        p.error('trajectoryまたは--scene-onlyが必要です')
    trajectory=np.load(args.trajectory)
    names=trajectory['joint_names'].tolist()
    controlled=[n for n in names if (n.startswith(('arm_l_joint','arm_r_joint','left_','right_')) or n == 'lift_joint') and 'wheel' not in n]
    known={model.joint(i).name:i for i in range(model.njnt)}
    missing=[n for n in controlled if n not in known]
    if missing: raise ValueError(f'Model is missing joints: {missing}')
    joint_ids=np.array([known[n] for n in controlled])
    addresses=model.jnt_qposadr[joint_ids]
    columns=[names.index(n) for n in controlled]
    positions=trajectory['positions'][:,columns]
    if not np.isfinite(positions).all(): raise ValueError('nonfinite joint positions')
    bounds=model.jnt_range[joint_ids]
    clipped=np.clip(positions,bounds[:,0],bounds[:,1])
    base=data.qpos.copy()
    def frame(i):
        data.qpos[:]=base;data.qpos[addresses]=clipped[i];data.qvel[:]=0
        mujoco.mj_forward(model,data)
    if args.check:
        contacts=[]; penetration=[]; table_penetration=[]
        table_id=model.geom('table_top').id
        for i in range(len(positions)):
            frame(i)
            contacts.append(data.ncon)
            penetration.append(max([0.]+[-float(c.dist) for c in data.contact]))
            table_penetration.append(max([0.]+[-float(c.dist) for c in data.contact if table_id in (c.geom1,c.geom2)]))
            if not np.isfinite(data.xpos).all(): raise ValueError(f'nonfinite frame {i}')
        report=dict(trajectory=str(args.trajectory.resolve()),mode='MuJoCo kinematic playback (mj_forward), not physics or policy execution',
            controlled_joints=len(controlled),frames=len(positions),duration_s=float(trajectory['time_s'][-1]),
            additional_model_limit_clamps=int(np.count_nonzero(clipped!=positions)),
            contact_count_max=int(max(contacts)),max_scene_penetration_m=float(max(penetration)),max_table_penetration_m=float(max(table_penetration)),
            note='Scene includes cube, a 700 mm table and printed ArUco board; contacts include scene contacts and do not establish task success. Scene object placement is assumed; recorded RGB is not used as the simulation camera.')
        target=args.trajectory.with_suffix('.mujoco.json');target.write_text(json.dumps(report,ensure_ascii=False,indent=2))
        print(json.dumps(report,ensure_ascii=False,indent=2));return
    from mujoco import viewer as mj_viewer
    frame(0)
    with mj_viewer.launch_passive(model,data) as viewer:
        viewer.cam.lookat[:]=[.1,0,.8];viewer.cam.distance=2.3;viewer.cam.azimuth=145;viewer.cam.elevation=-18
        while viewer.is_running():
            start=time.monotonic()
            while viewer.is_running():
                elapsed=time.monotonic()-start
                i=min(len(positions)-1,max(0,np.searchsorted(trajectory['time_s'],elapsed,side='right')-1))
                with viewer.lock():
                    frame(i)
                    targets=trajectory.get('wrist_target_positions_m',trajectory.get('controller_target_positions_m'))
                    if args.show_targets and targets is not None and len(targets):
                        viewer.user_scn.ngeom=2
                        for side in range(2):
                            mujoco.mjv_initGeom(viewer.user_scn.geoms[side],
                                type=mujoco.mjtGeom.mjGEOM_SPHERE,size=np.array([.018]*3),
                                pos=targets[i,side],mat=np.eye(3).ravel(),
                                rgba=np.array([0,1,1,.8] if side==0 else [1,0,1,.8]))
                viewer.sync();time.sleep(.02)
                if elapsed>trajectory['time_s'][-1]+2: break
            if not args.loop: break

if __name__=='__main__':main()
