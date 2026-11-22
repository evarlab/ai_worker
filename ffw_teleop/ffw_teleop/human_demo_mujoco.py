"""Preview exported human joint trajectories using the existing AI Worker MuJoCo model.

Kinematic replay via mj_forward: no inferred object interaction or trained policy.
"""
import argparse
import json
from pathlib import Path
import time
import numpy as np
import mujoco
from ffw_sharpa_sim.model import load


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('trajectory',type=Path)
    p.add_argument('--loop',action='store_true')
    p.add_argument('--check',action='store_true',help='headless: validate every frame and save report')
    args=p.parse_args()
    trajectory=np.load(args.trajectory)
    model,data=load()
    names=trajectory['joint_names'].tolist()
    controlled=[n for n in names if n.startswith(('arm_l_joint','arm_r_joint','left_','right_')) and 'wheel' not in n]
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
        contacts=[]; penetration=[]
        for i in range(len(positions)):
            frame(i)
            contacts.append(data.ncon)
            penetration.append(max([0.]+[-float(c.dist) for c in data.contact]))
            if not np.isfinite(data.xpos).all(): raise ValueError(f'nonfinite frame {i}')
        report=dict(trajectory=str(args.trajectory.resolve()),mode='MuJoCo kinematic playback (mj_forward), not physics or policy execution',
            controlled_joints=len(controlled),frames=len(positions),duration_s=float(trajectory['time_s'][-1]),
            additional_model_limit_clamps=int(np.count_nonzero(clipped!=positions)),
            contact_count_max=int(max(contacts)),max_scene_penetration_m=float(max(penetration)),
            note='Scene includes default cube/table; contacts include scene contacts and do not establish task success. No task objects or camera were recorded.')
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
                with viewer.lock(): frame(i)
                viewer.sync();time.sleep(.02)
                if elapsed>trajectory['time_s'][-1]+2: break
            if not args.loop: break

if __name__=='__main__':main()
