"""Measured controller mounts and a stationary board reference; never guess transforms."""
from collections import deque
import json
from pathlib import Path
import time
import numpy as np
from scipy.spatial.transform import Rotation
from .human_demo_data import rigid


def mean_transform(transforms):
    values=np.asarray([rigid(t) for t in transforms])
    result=np.eye(4)
    result[:3,:3]=Rotation.from_matrix(values[:,:3,:3]).mean().as_matrix()
    result[:3,3]=np.median(values[:,:3,3],axis=0)
    return result


def residuals(transforms, centre):
    errors=np.array([np.linalg.inv(centre)@rigid(t) for t in transforms])
    return np.linalg.norm(errors[:,:3,3],axis=1), Rotation.from_matrix(errors[:,:3,:3]).magnitude()


def fit_mount(pairs, min_samples=30):
    if len(pairs)<min_samples:
        raise ValueError(f'手首とコントローラの同時追跡が不足しています ({len(pairs)}/{min_samples})')
    transforms=np.array([np.linalg.inv(rigid(c))@rigid(w) for c,w in pairs])
    centre=mean_transform(transforms)
    distances,angles=residuals(transforms,centre)
    keep=(distances<=.015)&(angles<=np.deg2rad(12))
    if int(keep.sum())<min_samples or keep.mean()<.7:
        raise ValueError('取り付け変換のばらつきが大きすぎます。手を開き、遮蔽を減らして取り直してください')
    centre=mean_transform(transforms[keep])
    distances,angles=residuals(transforms[keep],centre)
    if np.linalg.norm(centre[:3,3])>.4:
        raise ValueError('手首とコントローラが40cm以上離れています')
    return dict(T_controller_wrist=centre.tolist(),samples=int(keep.sum()),
        rejected_samples=int((~keep).sum()),position_p95_m=float(np.percentile(distances,95)),
        rotation_p95_deg=float(np.rad2deg(np.percentile(angles,95))),
        source='WebXR optical wrist, same XRFrame as controller',
        note='Approximate anatomical mount; optical hand tracking is not ground truth.')


def fit_board(samples):
    """Solve H_world_head * X_head_camera * B_camera_board = constant C_world_board."""
    import cv2
    if len(samples)<20:
        raise ValueError(f'マーカーと頭の同時サンプルが不足しています ({len(samples)}/20)')
    heads=np.array([rigid(h) for h,b in samples]);boards=np.array([rigid(b) for h,b in samples])
    motion=Rotation.from_matrix(heads[0,:3,:3].T@heads[:,:3,:3]).as_rotvec()
    axes=np.linalg.svd(motion-motion.mean(axis=0),compute_uv=False)/np.sqrt(len(motion))
    if axes[1]<np.deg2rad(4):
        raise ValueError('頭の回転方向が不足しています。紙を固定し、上下と左右へゆっくり向きを変えてください')
    def solve(h,b):
        r,t=cv2.calibrateHandEye(list(h[:,:3,:3]),list(h[:,:3,3]),list(b[:,:3,:3]),list(b[:,:3,3]),method=cv2.CALIB_HAND_EYE_PARK)
        x=np.eye(4);x[:3,:3]=r;x[:3,3]=np.asarray(t).ravel()
        return rigid(x)
    x=solve(heads,boards)
    if not np.isfinite(x).all() or np.linalg.norm(x[:3,3])>.3:
        raise ValueError('頭→カメラ変換が不正です。回転方向を増やして取り直してください')
    world_boards=heads@x@boards; centre=mean_transform(world_boards)
    d,a=residuals(world_boards,centre);keep=(d<.025)&(a<np.deg2rad(4))
    if keep.sum()<20 or keep.mean()<.7:
        raise ValueError('紙の推定位置が安定しません。紙を固定し、頭をゆっくり動かして取り直してください')
    x=solve(heads[keep],boards[keep]);world_boards=heads[keep]@x@boards[keep]
    centre=mean_transform(world_boards);d,a=residuals(world_boards,centre)
    if np.percentile(d,95)>.02 or np.percentile(a,95)>np.deg2rad(3):
        raise ValueError('ボード校正の残差が許容値を超えています')
    return dict(T_head_camera=x.tolist(),T_world_board=centre.tolist(),samples=int(keep.sum()),
        position_p95_m=float(np.percentile(d,95)),rotation_p95_deg=float(np.rad2deg(np.percentile(a,95))),
        method='stationary printed board + multi-axis head motion hand-eye fit',
        timing='Browser camera callback and WebXR timestamps approximately matched; not exposure synchronized')


class Calibration:
    def __init__(self,save_path=None):
        self.save_path=Path(save_path).expanduser() if save_path else None
        self.reference_id=None
        self.mounts={}
        self.mount_pairs={side:deque(maxlen=450) for side in ('left','right')}
        self.xr=deque(maxlen=300)
        self.board=None;self.board_samples=[];self.collect_board=False;self.board_camera=None
        self.hand_status={}
        self.last_board=None
        self.anchor_samples=deque(maxlen=30)
        self.camera_extrinsic=None
        if self.save_path and self.save_path.exists():
            saved=json.loads(self.save_path.read_text())
            self.mounts={side:value for side,value in saved.get('mounts',{}).items()
                         if value.get('source')=='manual_model_alignment'}
            self.camera_extrinsic=saved.get('camera_extrinsic')
            if self.camera_extrinsic:
                rigid(self.camera_extrinsic['T_head_camera'])

    def reset_reference(self,reference_id):
        if reference_id!=self.reference_id:
            self.reference_id=reference_id;self.board=None;self.board_samples=[]
            self.collect_board=False;self.board_camera=None;self.xr.clear();self.last_board=None;self.anchor_samples.clear()
            for samples in self.mount_pairs.values():samples.clear()

    def observe_xr(self,data):
        self.reset_reference(data['reference_id'])
        now=time.monotonic();self.xr.append((now,dict(data)))
        fields={};self.hand_status={}
        for side in ('left','right'):
            hand=data.get('hands',{}).get(side,{})
            controller=data.get(side)
            wrist=hand.get('wrist')
            usable=controller is not None and wrist is not None and hand.get('emulated_position') is False and not data.get('controllers',{}).get(side,{}).get('emulated_position',False)
            self.hand_status[side]=dict(wrist_tracked=wrist is not None,controller_tracked=controller is not None,simultaneous=usable)
            if usable:
                c=rigid(np.asarray(controller).reshape(4,4,order='F'))
                w=rigid(np.asarray(wrist).reshape(4,4,order='F'))
                self.mount_pairs[side].append((now,c,w))
            if self.board is not None and controller is not None:
                c=rigid(np.asarray(controller).reshape(4,4,order='F'))
                bc=np.linalg.inv(self.board['T_world_board'])@c
                fields['T_board_'+side+'_controller']=bc.tolist()
                if side in self.mounts:
                    fields['T_board_'+side+'_wrist']=(bc@self.mounts[side]['T_controller_wrist']).tolist()
        return fields

    def observe_board(self,detection,metadata):
        pose=detection.get('pose')
        self.last_board=None if pose is None else dict(marker_ids=pose['marker_ids'],reprojection_px=pose['reprojection_px'])
        if pose is None:return
        candidates=[d for _,d in self.xr if d.get('reference_id')==metadata.get('reference_id') and d.get('head') is not None and 'client_ms' in d]
        if not candidates:return
        closest=min(candidates,key=lambda d:abs(d['client_ms']-metadata['client_ms']))
        skew=abs(closest['client_ms']-metadata['client_ms'])
        if skew>50:return
        camera=(metadata.get('camera_label'),detection['width'],detection['height'],
                metadata.get('intrinsics_source',{}).get('serial'))
        head=rigid(np.array(closest['head']).reshape(4,4,order='F'))
        if not self.collect_board:
            saved=self.camera_extrinsic
            if self.board is None and saved and tuple(saved.get('camera',()))==camera and camera[3]:
                self.anchor_samples.append(head@rigid(saved['T_head_camera'])@rigid(pose['T_camera_board']))
                if len(self.anchor_samples)>=20:
                    centre=mean_transform(self.anchor_samples);d,a=residuals(self.anchor_samples,centre)
                    if np.percentile(d,95)<.02 and np.percentile(a,95)<np.deg2rad(3):
                        self.board=dict(T_head_camera=saved['T_head_camera'],T_world_board=centre.tolist(),
                            reference_id=self.reference_id,camera=camera,samples=len(self.anchor_samples),
                            position_p95_m=float(np.percentile(d,95)),rotation_p95_deg=float(np.rad2deg(np.percentile(a,95))),
                            method='Re-anchor stationary board using previously measured head-camera transform')
                        self.save()
                        return self.board
            return
        if self.board_camera is not None and self.board_camera!=camera:
            raise ValueError('校正中にカメラまたは解像度が変わりました。board-startで取り直してください')
        self.board_camera=camera
        self.board_samples.append((rigid(np.array(closest['head']).reshape(4,4,order='F')),rigid(pose['T_camera_board'])))
        self.board_samples=self.board_samples[-600:]

    def calibrate_mount(self):
        now=time.monotonic()
        results={side:fit_mount([(c,w) for t,c,w in samples if now-t<10]) for side,samples in self.mount_pairs.items()}
        self.mounts=results;self.save();return results

    def set_manual_mount(self, side, transform, model, profile_uuid=None):
        if side not in ('left','right'):
            raise ValueError('side must be left or right')
        if model not in ('meta-quest-touch-pro','meta-quest-touch-plus'):
            raise ValueError('Unknown controller model')
        matrix=rigid(transform)
        if np.linalg.norm(matrix[:3,3])>.5:
            raise ValueError('取り付け位置は追跡原点から50cm以内にしてください')
        result=dict(T_controller_wrist=matrix.tolist(),source='manual_model_alignment',
            target_frame='contactglove_skeleton_root',controller_model=model,
            skeleton_profile_uuid=profile_uuid,position_unit='m',
            axes='+X fingertips toward wrist; +Y palm toward back; Z across hand',
            note='User-adjusted visual alignment, not an optical measurement.')
        self.mounts[side]=result;self.save();return result

    def begin_board(self):
        self.board_samples=[];self.board_camera=None;self.collect_board=True
        return '紙を固定し、マーカーを見ながら頭を上下・左右へゆっくり回転してください。その後board-finish。'

    def finish_board(self):
        result=fit_board(self.board_samples)
        result.update(reference_id=self.reference_id,camera=self.board_camera)
        self.camera_extrinsic=dict(T_head_camera=result['T_head_camera'],camera=self.board_camera)
        self.board=result;self.collect_board=False;self.save();return result

    def snapshot(self):
        return dict(schema='human_calibration_v1',mounts=self.mounts,board=self.board,camera_extrinsic=self.camera_extrinsic)

    def save(self):
        if self.save_path:
            self.save_path.parent.mkdir(parents=True,exist_ok=True)
            temp=self.save_path.with_suffix('.tmp');temp.write_text(json.dumps(self.snapshot(),ensure_ascii=False,indent=2));temp.replace(self.save_path)

    def status(self):
        now=time.monotonic()
        return dict(hands=self.hand_status,mount_samples={side:sum(now-t<10 for t,_,_ in samples) for side,samples in self.mount_pairs.items()},
            mount_calibrated=list(self.mounts),board_collecting=self.collect_board,board_samples=len(self.board_samples),
            board_calibrated=self.board is not None,last_board=self.last_board)
