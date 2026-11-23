"""Robot-independent human demonstrations, preserving clocks and reference resets."""
from collections import Counter, deque
from datetime import datetime
import gzip
import json
from pathlib import Path
import threading
import time

import numpy as np


def rigid(value):
    matrix = np.asarray(value, dtype=float).reshape(4, 4)
    if not np.isfinite(matrix).all() or not np.allclose(matrix[3], [0, 0, 0, 1], atol=1e-5):
        raise ValueError('SE(3) matrix is invalid')
    rotation = matrix[:3, :3]
    if not np.allclose(rotation.T@rotation, np.eye(3), atol=1e-4) or not np.isclose(np.linalg.det(rotation), 1., atol=1e-4):
        raise ValueError('Rotation must be right-handed and orthonormal')
    return matrix


def task_frame(origin, right, forward):
    """T_world_task; task X=right, Y=forward, Z=up (no reflections)."""
    origin, right, forward = (np.asarray(x, dtype=float) for x in (origin, right, forward))
    x, y = right-origin, forward-origin
    if not np.isfinite([origin, right, forward]).all() or np.linalg.norm(x) < .03:
        raise ValueError('基準点は3cm以上離してください')
    x /= np.linalg.norm(x)
    y -= np.dot(x, y)*x
    if np.linalg.norm(y) < .03:
        raise ValueError('前方の基準点が近すぎるか、3点が一直線です')
    y /= np.linalg.norm(y)
    z = np.cross(x, y)
    if z[1] < .5:  # WebXR gravity: +Y is up. Catch swapped right/forward points.
        raise ValueError('基準の左右・前後が逆か、台面の傾きが大きすぎます')
    result = np.eye(4)
    result[:3, :3] = np.column_stack((x, y, z))
    result[:3, 3] = origin
    return rigid(result)


class HumanEpisodes:
    def __init__(self, root, workspace=None):
        self.capture_calibration = None
        self.workspace = workspace
        self.root = Path(root).expanduser()
        self.root.mkdir(parents=True, exist_ok=True)
        self.lock = threading.RLock()
        self.stream = None
        self.directory = None
        self.latest = {}
        self.counts = Counter()
        self.recent_xr = deque(maxlen=180)
        self.xr_received = 0
        self.reference_id = None
        self.calibration = {}
        self.marks = {}
        self.following = False
        self.head_origin = None
        self.flush_time = time.monotonic()

    def follow_start(self):
        with self.lock:
            if self.stream is not None:
                raise ValueError('記録中は初期位置を変更できません。先にstopしてください')
            sample = self.latest.get('xr')
            if not sample or time.monotonic_ns()-sample['receive_monotonic_ns'] > 300_000_000:
                raise ValueError('Questの新しい頭の位置データがありません')
            data = sample['data']
            if data.get('reference_id') != self.reference_id or data.get('head') is None:
                raise ValueError('頭の追跡またはreference_idを確認してください')
            head = rigid(np.asarray(data['head']).reshape(4, 4, order='F'))
            self.head_origin = dict(reference_id=self.reference_id,
                T_world_head_origin=head.tolist(), receive_monotonic_ns=sample['receive_monotonic_ns'],
                hand_coordinates='current_head', head_coordinates='head_at_follow_start')
            self.following = True
            self.record('follow_start', self.head_origin.copy())
            return self.head_origin.copy()

    def follow_stop(self):
        with self.lock:
            directory = self.stop('interrupted')
            self.following = False
            self.head_origin = None
            self.record('follow_stop', {})
            return directory

    def start(self, task='unspecified'):
        with self.lock:
            if self.stream is not None:
                raise ValueError('既に収集中です')
            self.directory = self.root/f'{datetime.now():%Y%m%d-%H%M%S}-{time.time_ns()}'
            self.directory.mkdir()
            self.counts = Counter()
            self.manifest = dict(capture_calibration=self.capture_calibration.snapshot() if self.capture_calibration else None, workspace=self.workspace, schema='human_demo_v1', task=task, complete=False,
                started_wall_ns=time.time_ns(), units=dict(position='m', glove='degrees'),
                glove_angle_order=['thumb.cmc', 'thumb.mcp', 'thumb.ip', 'thumb.tilt']+
                    [finger+'.'+joint for finger in ('index', 'middle', 'ring', 'little') for joint in ('mcp', 'pip', 'dip', 'tilt')],
                coordinates='WebXR right-handed +X right +Y up -Z forward; matrices column-major in raw WebXR packets',
                reference_id=self.reference_id, calibration=self.calibration.copy(),
                head_origin=self.head_origin.copy() if self.following else None,
                camera_streams={}, note='Human data only. No robot joint angles, currents, depth, or policy are inferred.')
            self._manifest()
            self.stream = gzip.open(self.directory/'events.jsonl.gz', 'wt', encoding='utf-8', compresslevel=1)
            self.record('episode_start', dict(reference_id=self.reference_id))
            # Snapshot existing calibration/reference metadata, never stale poses.
            for kind in ('reference', 'clock_sync', 'camera_status', 'capabilities'):
                if kind in self.latest:
                    self.record(kind, self.latest[kind]['data'])
            return str(self.directory)

    def _manifest(self):
        target = self.directory/'manifest.json'
        temporary = target.with_suffix('.tmp')
        temporary.write_text(json.dumps(self.manifest, ensure_ascii=False, indent=2))
        temporary.replace(target)

    def record(self, kind, data):
        with self.lock:
            event = dict(event=kind, receive_monotonic_ns=time.monotonic_ns(), receive_wall_ns=time.time_ns(), data=data)
            self.latest[kind] = event
            if kind == 'xr':
                self.xr_received += 1
                self.recent_xr.append(event['receive_monotonic_ns'])
                data = dict(data, following=self.following)
                if self.capture_calibration:
                    data.update(self.capture_calibration.observe_xr(data))
                event['data'] = data
                if self.following and self.head_origin and data.get('reference_id') == self.head_origin['reference_id']:
                    head = rigid(np.asarray(data['head']).reshape(4, 4, order='F')) if data.get('head') is not None else None
                    data['T_head_origin_head'] = (np.linalg.inv(self.head_origin['T_world_head_origin'])@head).tolist() if head is not None else None
                    for side in ('left', 'right'):
                        valid = head is not None and data.get(side) is not None and not data.get('controllers', {}).get(side, {}).get('emulated_position')
                        data['T_head_'+side] = (np.linalg.inv(head)@rigid(np.asarray(data[side]).reshape(4, 4, order='F'))).tolist() if valid else None
            if kind == 'reference':
                new_id = data.get('reference_id')
                if self.capture_calibration:
                    self.capture_calibration.reset_reference(new_id)
                if new_id != self.reference_id:
                    if self.following:
                        self.follow_stop()
                    self.calibration = {}
                    self.marks = {}
                self.reference_id = new_id
            self.latest[kind] = event
            if self.stream is not None:
                self.stream.write(json.dumps(event, ensure_ascii=False, allow_nan=False)+'\n')
                self.counts[kind] += 1
                if time.monotonic()-self.flush_time > 1.:
                    self.stream.flush()
                    self.flush_time = time.monotonic()
            return event

    def mark(self, name, side='right'):
        with self.lock:
            if self.stream is not None and self.calibration:
                raise ValueError('収集中の作業基準の変更はできません。stopしてから取り直してください')
            if name not in ('origin', 'right', 'forward'):
                raise ValueError('markはorigin/right/forward')
            sample = self.latest.get('xr')
            if not sample or (time.monotonic_ns()-sample['receive_monotonic_ns'])/1e9 > .3:
                raise ValueError('Questの新しい位置データがありません')
            data = sample['data']
            pose = data.get(side)
            if data.get('reference_id') != self.reference_id or pose is None:
                raise ValueError('手の追跡またはreference_idを確認してください')
            self.marks[name] = rigid(np.array(pose).reshape(4, 4, order='F'))[:3, 3].tolist()
            if len(self.marks) == 3:
                transform = task_frame(*(self.marks[k] for k in ('origin', 'right', 'forward')))
                self.calibration = dict(reference_id=self.reference_id, T_world_task=transform.tolist(),
                    marks=self.marks.copy(), pointer=side+'_controller_grip',
                    note='Controller grip origin, not fingertip. Use the same controller orientation at all three points.')
                self.record('task_calibration', self.calibration)
            return self.calibration or self.marks

    def image(self, camera, content, metadata):
        with self.lock:
            if self.stream is None:
                raise ValueError('先に記録を開始してください')
            if not camera.replace('_', '').replace('-', '').isalnum():
                raise ValueError('Invalid camera name')
            if not content.startswith(b'\xff\xd8'):
                raise ValueError('JPEG is required')
            image_dir = self.directory/'images'/camera
            image_dir.mkdir(parents=True, exist_ok=True)
            filename = image_dir/f'{time.monotonic_ns()}.jpg'
            filename.write_bytes(content)
            self.manifest['camera_streams'][camera] = metadata
            self.record('image', dict(camera=camera, path=str(filename.relative_to(self.directory)), metadata=metadata))

    def stop(self, outcome='unspecified'):
        with self.lock:
            if self.stream is None:
                return None
            self.record('episode_end', dict(outcome=outcome))
            self.stream.close()
            self.stream = None
            self.manifest.update(complete=True, outcome=outcome, finished_wall_ns=time.time_ns(), counts=dict(self.counts))
            self._manifest()
            return str(self.directory)

    def status(self):
        with self.lock:
            xr_event = self.latest.get('xr', {})
            xr = xr_event.get('data', {})
            age = (time.monotonic_ns()-xr_event.get('receive_monotonic_ns', 0))/1e9
            tracking = {}
            head = rigid(np.asarray(xr['head']).reshape(4, 4, order='F')) if xr.get('head') is not None else None
            for side in ('left', 'right'):
                controller = xr.get('controllers', {}).get(side, {})
                tracked = age < .5 and xr.get(side) is not None and not controller.get('emulated_position', False)
                behind = None
                relative_position = None
                if tracked and head is not None:
                    hand = rigid(np.asarray(xr[side]).reshape(4, 4, order='F'))
                    relative_position = (np.linalg.inv(head)@hand)[:3, 3].tolist()
                    behind = bool(relative_position[2] > .1)
                tracking[side] = dict(tracked=tracked, behind_head=behind,
                    head_relative_position_m=relative_position,
                    profiles=controller.get('profiles', []), emulated_position=controller.get('emulated_position'))
            duration = (self.recent_xr[-1]-self.recent_xr[0])/1e9 if len(self.recent_xr) > 1 else 0.
            return dict(capture_calibration=self.capture_calibration.status() if self.capture_calibration else None, recording=self.stream is not None, following=self.following, head_origin=self.head_origin, directory=str(self.directory) if self.directory else None,
                reference_id=self.reference_id, calibrated=bool(self.calibration), counts=dict(self.counts),
                tracking=tracking, xr_received=self.xr_received,
                xr_rate_hz=(len(self.recent_xr)-1)/duration if duration and age < .5 else 0.,
                ages_s={k: (time.monotonic_ns()-v['receive_monotonic_ns'])/1e9 for k, v in self.latest.items()})
