"""Offline relative-motion IK and isolated URDF playback; no hardware commands."""
import argparse
import json
from pathlib import Path
import time
import xml.etree.ElementTree as ET
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
from .contactglove_mapping import FLEXION, ABDUCTION


class Arm:
    def __init__(self, root, side, fixed_joints=None):
        self.fixed_joints = dict(fixed_joints or {})
        self.names = [f'arm_{side}_joint{i}' for i in range(1, 8)]
        joints = {j.find('child').get('link'): j for j in root.findall('joint')}
        chain, link = [], f'arm_{side}_link7'
        while link in joints:
            j = joints[link]
            origin = j.find('origin')
            t = np.eye(4)
            if origin is not None:
                t[:3, 3] = np.fromstring(origin.get('xyz', '0 0 0'), sep=' ')
                t[:3, :3] = Rotation.from_euler('xyz', np.fromstring(origin.get('rpy', '0 0 0'), sep=' ')).as_matrix()
            axis = j.find('axis')
            chain.append((j.get('name'), t, np.fromstring(axis.get('xyz') if axis is not None else '1 0 0', sep=' '), j.get('type')))
            link = j.find('parent').get('link')
        self.chain = list(reversed(chain))
        byname = {j.get('name'): j for j in root.findall('joint')}
        self.low = np.array([float(byname[n].find('limit').get('lower', '-3.14')) for n in self.names])
        self.high = np.array([float(byname[n].find('limit').get('upper', '3.14')) for n in self.names])

    def fk(self, q):
        values = {**self.fixed_joints, **dict(zip(self.names, q))}
        t = np.eye(4)
        for name, origin, axis, kind in self.chain:
            t = t @ origin
            move = np.eye(4)
            if kind in ('revolute', 'continuous'):
                move[:3, :3] = Rotation.from_rotvec(axis*values.get(name, 0.)).as_matrix()
            elif kind == 'prismatic':
                move[:3, 3] = axis*values.get(name, 0.)
            t = t @ move
        return t


def floor_preview_positions(world_controller, basis, first, forward_m=.50):
    """One shared rigid translation; retain WebXR local-floor height and hand spacing.

    Only horizontal placement is assumed: first hand midpoint at forward_m, y=0.
    This is not measured ArUco alignment.
    """
    points = np.einsum('ij,nsj->nsi', basis, world_controller[:, :, :3, 3])
    midpoint = points[first].mean(axis=0)
    offset = np.array([forward_m-midpoint[0], -midpoint[1], 0.])
    return points+offset


def convert(arrays, urdf, output, hz=10., alignment='floor', forward_m=.35, lift_m=-.15):
    data = np.load(arrays)
    root = ET.parse(urdf).getroot()
    names = [j.get('name') for j in root.findall('joint') if j.get('type') != 'fixed']
    limits = {j.get('name'): (float(j.find('limit').get('lower', '-1000000')), float(j.find('limit').get('upper', '1000000'))) for j in root.findall('joint') if j.find('limit') is not None}
    ns = data['receive_monotonic_ns']; times = (ns-ns[0])/1e9
    valid = data['pose_valid'].all(axis=1) & data['head_valid'].all(axis=1) & data['glove_valid'].all(axis=1)
    if len(set(data['reference_id'])) != 1:
        raise ValueError('reference resetを含みます。区間を分けてください')
    indices = []; last = -1e9
    for i, t in enumerate(times):
        if valid[i] and t-last >= 1/hz:
            indices.append(i); last = t
    if not indices:
        raise ValueError('有効な両手サンプルがありません')
    first = indices[0]
    world_controller = data['T_world_controller']
    head = world_controller[first, 0] @ np.linalg.inv(data['T_head_controller'][first, 0])
    forward = -head[:3, 2]; forward[1] = 0; forward /= np.linalg.norm(forward)
    up = np.array([0., 1., 0.]); left = np.cross(up, forward)
    basis = np.stack([forward, left, up])  # WebXR world -> robot X forward Y left Z up
    targets = floor_preview_positions(world_controller, basis, first, forward_m)
    result = np.zeros((len(indices), len(names)))
    if 'lift_joint' in names:
        lo, hi = limits['lift_joint']
        if not np.isfinite(lift_m) or not lo <= lift_m <= hi:
            raise ValueError('lift is outside the URDF limits')
        result[:, names.index('lift_joint')] = lift_m
    errors = np.zeros((len(indices), 2, 2)); clipped = [0, 0]
    import yaml
    cfg_path = Path('~/.config/ai_worker/contactglove_sharpa.yaml').expanduser()
    cfg = yaml.safe_load(cfg_path.read_text())['contactglove_sharpa']['ros__parameters'] if cfg_path.exists() else {}
    for side_index, side in enumerate(('left', 'right')):
        arm = Arm(root, 'l' if side == 'left' else 'r', {'lift_joint': lift_m})
        q = np.clip(np.zeros(7), arm.low+1e-6, arm.high-1e-6)
        q[4] = np.clip(-np.pi/2 if side == 'left' else np.pi/2, arm.low[4], arm.high[4])
        start_robot = arm.fk(q); start_human = world_controller[first, side_index]
        for k, i in enumerate(indices):
            human = world_controller[i, side_index]
            target = start_robot.copy()
            if alignment == 'floor':
                target[:3, 3] = targets[i, side_index]
            else:
                target[:3, 3] += basis @ (human[:3, 3]-start_human[:3, 3])
            target[:3, :3] = basis @ human[:3, :3] @ start_human[:3, :3].T @ basis.T @ start_robot[:3, :3]
            old = q.copy()
            def residual(x):
                pose = arm.fk(x)
                return np.r_[pose[:3, 3]-target[:3, 3], .15*Rotation.from_matrix(target[:3, :3] @ pose[:3, :3].T).as_rotvec(), .005*(x-old)]
            seeds = [q]
            if k == 0:
                bent = q.copy(); bent[0] = -.3; bent[1] = .25 if side == 'left' else -.25
                bent[3] = -1.6; bent[5] = .2
                seeds.append(bent)
            fits = [least_squares(residual, np.clip(seed, arm.low+1e-8, arm.high-1e-8),
                    bounds=(arm.low, arm.high), max_nfev=150 if k == 0 else 75, ftol=1e-5) for seed in seeds]
            q = min(fits, key=lambda fit: np.linalg.norm(residual(fit.x))).x
            actual = arm.fk(q)
            errors[k, side_index] = [np.linalg.norm(actual[:3, 3]-target[:3, 3]), Rotation.from_matrix(target[:3, :3] @ actual[:3, :3].T).magnitude()]
            for n, v in zip(arm.names, q): result[k, names.index(n)] = v
            angles = data['glove_angles_degrees'][i, side_index]
            for mapping, prefix in ((FLEXION, ''), (ABDUCTION, 'abduction_')):
                for j, (joint, index, _) in enumerate(mapping):
                    n = f'{side}_{joint}'
                    if n not in names: continue
                    value = np.deg2rad(angles[index])*cfg.get(f'{side}_{prefix}gains', [1.]*len(mapping))[j]+cfg.get(f'{side}_{prefix}offsets', [0.]*len(mapping))[j]
                    lo, hi = limits[n]; clipped[side_index] += int(value < lo or value > hi)
                    result[k, names.index(n)] = np.clip(value, lo, hi)
        print(f'{side}: {len(indices)} frames solved', flush=True)
    output = Path(output)
    np.savez_compressed(output, time_s=times[indices]-times[first], joint_names=np.array(names), positions=result, ik_errors=errors,
        controller_target_positions_m=targets[indices] if alignment == 'floor' else np.empty((0,2,3)))
    report = dict(source=str(Path(arrays).resolve()), urdf=str(Path(urdf).resolve()), frames=len(indices), duration_s=float(times[indices[-1]]-times[first]),
        side_order=['left','right'], position_error_p50_m=np.median(errors[:,:,0],axis=0).tolist(), position_error_p95_m=np.percentile(errors[:,:,0],95,axis=0).tolist(), rotation_error_p95_deg=np.rad2deg(np.percentile(errors[:,:,1],95,axis=0)).tolist(),
        reachable_fraction_5cm_20deg=((errors[:,:,0]<.05)&(errors[:,:,1]<np.deg2rad(20))).mean(axis=0).tolist(), finger_clamped_values=clipped,
        alignment=('Shared first-head yaw and horizontal translation; WebXR local-floor height and hand separation preserved. '
                   f'Initial hand midpoint placed at x={forward_m}m, y=0. No measured task/mount calibration.'
                   if alignment == 'floor' else 'Legacy independent first-controller alignment to relaxed arms.'),
        lift_joint_m=lift_m,
        mode='Kinematic model preview only; no physics, contacts, collision validation, policy training or hardware deployment. Invalid samples are omitted; gaps hold last pose.',
        glove_config=cfg)
    output.with_suffix('.json').write_text(json.dumps(report, ensure_ascii=False, indent=2))
    print(json.dumps({k:v for k,v in report.items() if k!='glove_config'}, ensure_ascii=False, indent=2))


def play(trajectory, loop=False):
    import rclpy
    from sensor_msgs.msg import JointState
    rclpy.init(domain_id=97)
    node = rclpy.create_node('human_demo_sim_replay')
    pub = node.create_publisher(JointState, '/joint_states', 10)
    data = np.load(trajectory); clock = data['time_s']; positions = data['positions']
    print('MODEL ONLY ROS_DOMAIN_ID=97; Ctrl+C to stop', flush=True)
    try:
        while rclpy.ok():
            start = time.monotonic()
            while rclpy.ok():
                elapsed = time.monotonic()-start
                i = min(len(clock)-1, max(0, np.searchsorted(clock, elapsed, side='right')-1))
                msg = JointState(name=data['joint_names'].tolist(), position=positions[i].tolist())
                msg.header.stamp = node.get_clock().now().to_msg(); pub.publish(msg)
                rclpy.spin_once(node, timeout_sec=.02)
                if elapsed > clock[-1]+2: break
            if not loop: break
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node(); rclpy.try_shutdown()


def main():
    p = argparse.ArgumentParser(description=__doc__); sub = p.add_subparsers(dest='command', required=True)
    c = sub.add_parser('convert'); c.add_argument('arrays'); c.add_argument('--urdf', type=Path, default=Path('~/.cache/ai_worker/vr_preview/robot.urdf').expanduser()); c.add_argument('--output', required=True); c.add_argument('--hz', type=float, default=10.); c.add_argument('--alignment', choices=('floor','relative'), default='floor'); c.add_argument('--forward', type=float, default=.35); c.add_argument('--lift', type=float, default=-.15)
    r = sub.add_parser('play'); r.add_argument('trajectory'); r.add_argument('--loop', action='store_true')
    args = p.parse_args()
    if args.command == 'convert':
        if not np.isfinite(args.hz) or args.hz<=0: p.error('hz must be positive')
        convert(args.arrays,args.urdf,args.output,args.hz,args.alignment,args.forward,args.lift)
    else: play(args.trajectory,args.loop)

if __name__ == '__main__': main()
