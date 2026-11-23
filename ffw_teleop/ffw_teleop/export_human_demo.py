"""Export pose/ergonomics arrays and validity masks; never infer robot joints."""
import argparse
from bisect import bisect_right
import gzip
import json
from pathlib import Path

import numpy as np
from .human_demo_data import rigid


def export_episode(directory, max_age=.1, trim_start=0., trim_end=0., calibration_file=None):
    directory = Path(directory).expanduser().resolve()
    manifest = json.loads((directory/'manifest.json').read_text())
    if not manifest.get('complete'):
        raise ValueError('未完了のエピソードです。先にstopしてください')
    measured=json.loads(Path(calibration_file).read_text()) if calibration_file else {}
    measured_board=measured.get('board')
    measured_mounts=measured.get('mounts',{})
    if measured_board:
        rigid(measured_board['T_world_board'])
    for mount in measured_mounts.values():
        rigid(mount['T_controller_wrist'])
    calibration = {}
    initial = manifest.get('calibration', {})
    if initial:
        calibration[initial['reference_id']] = rigid(initial['T_world_task'])
    frames, gloves, images = [], {}, []
    previous = {}
    with gzip.open(directory/'events.jsonl.gz', 'rt') as stream:
        for line in stream:
            event = json.loads(line)
            data, kind = event['data'], event['event']
            if kind == 'task_calibration':
                calibration[data['reference_id']] = rigid(data['T_world_task'])
            elif kind in ('glove_left', 'glove_right'):
                gloves[kind.removeprefix('glove_')] = event
            elif kind in ('glove_invalid_left', 'glove_invalid_right'):
                gloves.pop(kind.removeprefix('glove_invalid_'), None)
            elif kind == 'image':
                images.append(event)
            elif kind == 'xr':
                frames.append((event, gloves.copy()))
    if not frames:
        raise ValueError('XRデータがありません')
    if measured_board and not any(e['data']['reference_id']==measured_board['reference_id'] for e,g in frames):
        raise ValueError('校正と記録のreference_idが一致しません')
    if not all(np.isfinite(v) and v >= 0 for v in (trim_start, trim_end)):
        raise ValueError('trim seconds must be finite and nonnegative')
    first_ns, last_ns = frames[0][0]['receive_monotonic_ns'], frames[-1][0]['receive_monotonic_ns']
    frames = [(e, g) for e, g in frames if first_ns+trim_start*1e9 <= e['receive_monotonic_ns'] <= last_ns-trim_end*1e9]
    if not frames:
        raise ValueError('前後を切るとデータが残りません')
    count = len(frames)
    poses = {key: np.full((count, 2, 4, 4), np.nan) for key in ('T_world_controller', 'T_head_controller', 'T_task_controller', 'delta_prev_controller', 'T_board_controller', 'T_board_wrist')}
    head_relative = np.full((count, 4, 4), np.nan)
    head_origin_valid = np.zeros(count, dtype=bool)
    angles = np.full((count, 2, 20), np.nan)
    glove_age = np.full((count, 2), np.nan)
    glove_source_ms = np.full((count, 2), -1, dtype=np.int64)
    masks = {key: np.zeros((count, 2), dtype=bool) for key in ('pose_valid', 'head_valid', 'task_valid', 'delta_valid', 'glove_valid', 'board_controller_valid', 'board_wrist_valid')}
    times, references = [], []
    image_paths = []
    image_age = []
    image_valid = []
    image_groups = {}
    for image in images:
        key = image['data']['metadata'].get('reference_id')
        image_groups.setdefault(key, []).append(image)
    image_times = {key: [e['receive_monotonic_ns'] for e in group] for key, group in image_groups.items()}
    for i, (event, glove_snapshot) in enumerate(frames):
        data, ns = event['data'], event['receive_monotonic_ns']
        ref = data['reference_id']
        times.append(ns)
        references.append(ref)
        head = rigid(np.array(data['head']).reshape(4, 4, order='F')) if data.get('head') is not None else None
        if data.get('T_head_origin_head') is not None and data.get('following'):
            head_relative[i] = rigid(data['T_head_origin_head'])
            head_origin_valid[i] = True
        for j, side in enumerate(('left', 'right')):
            for suffix, mask in (('controller','board_controller_valid'),('wrist','board_wrist_valid')):
                value=data.get('T_board_'+side+'_'+suffix)
                if value is not None:
                    poses['T_board_'+suffix][i,j]=rigid(value)
                    masks[mask][i,j]=True
            glove = glove_snapshot.get(side)
            if glove:
                age = (ns-glove['receive_monotonic_ns'])/1e9
                glove_age[i, j] = age
                glove_source_ms[i, j] = glove['data']['source_timestamp_ms']
                if 0 <= age <= max_age:
                    angles[i, j] = glove['data']['angles_degrees']
                    masks['glove_valid'][i, j] = True
            if data.get(side) is None or data.get('controllers', {}).get(side, {}).get('emulated_position'):
                previous.pop(side, None)
                continue
            pose = rigid(np.array(data[side]).reshape(4, 4, order='F'))
            poses['T_world_controller'][i, j] = pose
            masks['pose_valid'][i, j] = True
            if measured_board and ref==measured_board['reference_id']:
                board_controller=np.linalg.inv(measured_board['T_world_board'])@pose
                poses['T_board_controller'][i,j]=board_controller
                masks['board_controller_valid'][i,j]=True
                if side in measured_mounts:
                    poses['T_board_wrist'][i,j]=board_controller@measured_mounts[side]['T_controller_wrist']
                    masks['board_wrist_valid'][i,j]=True
            if head is not None:
                poses['T_head_controller'][i, j] = np.linalg.inv(head)@pose
                masks['head_valid'][i, j] = True
            if ref in calibration:
                poses['T_task_controller'][i, j] = np.linalg.inv(calibration[ref])@pose
                masks['task_valid'][i, j] = True
            old = previous.get(side)
            if old and old[0] == ref and 0 < ns-old[1] <= max_age*1e9:
                poses['delta_prev_controller'][i, j] = np.linalg.inv(old[2])@pose
                masks['delta_valid'][i, j] = True
            previous[side] = (ref, ns, pose)
        # Causal arrival-time match: never use an image received in the future.
        candidates = image_groups.get(ref, [])
        index = bisect_right(image_times.get(ref, []), ns)-1
        nearest = candidates[index] if index >= 0 else None
        age = (nearest['receive_monotonic_ns']-ns)/1e9 if nearest else float('nan')
        image_paths.append(nearest['data']['path'] if nearest else '')
        image_age.append(age)
        image_valid.append(nearest is not None and abs(age) <= max_age)
    output = directory/('human_arrays_trimmed.npz' if trim_start or trim_end else 'human_arrays.npz')
    np.savez_compressed(output, receive_monotonic_ns=np.array(times, dtype=np.int64),
        reference_id=np.array(references), glove_angles_degrees=angles,
        T_head_origin_head=head_relative, head_origin_valid=head_origin_valid,
        glove_age_s=glove_age, glove_source_timestamp_ms=glove_source_ms,
        image_path=np.array(image_paths), image_age_s=np.array(image_age), image_valid=np.array(image_valid),
        **poses, **masks)
    quality = dict(applied_capture_calibration=measured or None,frames=count, trim_start_s=trim_start, trim_end_s=trim_end, duration_s=(times[-1]-times[0])/1e9, source_duration_s=(last_ns-first_ns)/1e9, max_xr_gap_s=float(np.diff(times).max()/1e9) if len(times)>1 else 0., valid_counts={key: value.sum(axis=0).tolist() for key, value in masks.items()},
        image_matches=int(sum(image_valid)), matching='Causal PC arrival time, approximate; not exposure-synchronized',
        side_order=['left', 'right'], matrices='row-major 4x4 in NPZ; T_parent_child',
        action_note='delta_prev_controller is previous-to-current local SE(3), NOT a future action label or robot joint command',
        limitations=['Controller grip poses are not anatomical wrists without mount calibration.',
            'Head-relative poses are not camera-relative poses without camera extrinsics.',
            'No metric depth is inferred from RGB.', 'Human samples are not a deployable robot policy.'])
    output.with_suffix('.quality.json').write_text(json.dumps(quality, ensure_ascii=False, indent=2))
    if not trim_start and not trim_end:
        (directory/'export_quality.json').write_text(json.dumps(quality, ensure_ascii=False, indent=2))
    return output


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory', type=Path)
    parser.add_argument('--calibration',type=Path,help='同一reference_idの校正スナップショット')
    parser.add_argument('--max-age', type=float, default=.1)
    options = parser.parse_args(args)
    if not np.isfinite(options.max_age) or options.max_age <= 0:
        parser.error('max-ageは正の有限値です')
    print(export_episode(options.directory, options.max_age, calibration_file=options.calibration))


if __name__ == '__main__':
    main()
