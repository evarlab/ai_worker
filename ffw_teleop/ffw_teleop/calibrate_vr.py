"""Terminal wizard: three translations and three wrist rotations, no robot commands."""
import argparse
import json
from pathlib import Path
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
import yaml

from .vr_calibration_math import average_pose, fit_motion, validate_translation
from .vr_pose_math import pose_matrix


def atomic_yaml(path, data):
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + '.tmp')
    temporary.write_text(yaml.safe_dump(data, allow_unicode=True, sort_keys=False))
    temporary.replace(path)


def recover_recordings(path):
    """Re-fit complete hand recordings; never trust an old saved success flag."""
    results, recordings = {}, {}
    for line in path.read_text().splitlines():
        try:
            event = json.loads(line)
            if event.get('event') not in ('fit_rejected', 'fit_ok') or event.get('side') not in ('left', 'right'):
                continue
            translations = [tuple(np.asarray(p) for p in pair) for pair in event['translations']]
            rotations = [tuple(np.asarray(p) for p in pair) for pair in event['rotations']]
            fit = fit_motion(translations, rotations)
        except (ValueError, KeyError, TypeError):
            continue
        side = event['side']
        results[side] = fit
        recordings[side] = {key: event[key] for key in ('translations', 'rotations')}
    return results, recordings


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path,
        default=Path('~/.config/ai_worker/vr_motion_calibration.yaml').expanduser())
    parser.add_argument('--delay', type=float, default=5., help='Enter後、記録までの待ち時間[秒]')
    parser.add_argument('--resume-log', type=Path, help='同じVRセッションのJSONL記録から成立する手を復元して続行')
    options = parser.parse_args(args)
    if not 3 <= options.delay <= 30:
        parser.error('delayは3〜30秒')
    rclpy.init(args=[])
    node = rclpy.create_node('calibrate_sharpa_vr')
    audit_dir = Path('~/.local/state/ai_worker/calibration').expanduser()
    audit_dir.mkdir(parents=True, exist_ok=True)
    audit_path = audit_dir / f'poses-{time.time_ns()}.jsonl'

    def audit(event, **details):
        with audit_path.open('a') as stream:
            stream.write(json.dumps(dict(time=time.time(), event=event, **details), ensure_ascii=False)+'\n')

    print(f'計測ログ（不成立・中断時も保存）: {audit_path}', flush=True)
    samples = {'left': [], 'right': []}
    frame_seen = [0.]

    def on_frame(msg):
        if msg.data == 'world_v1':
            frame_seen[0] = time.monotonic()

    def on_pose(msg, side):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec/1e9
        if msg.header.frame_id != 'base_link' or abs(node.get_clock().now().nanoseconds/1e9-stamp) > .5:
            return
        try:
            matrix = pose_matrix(msg.pose)
        except ValueError:
            return
        history = samples[side]
        if history and stamp <= history[-1][1]:
            return
        history.append((time.monotonic(), stamp, matrix))
        del history[:-150]

    node.create_subscription(String, '/vr/input/reference_frame', on_frame, 10)
    for side in samples:
        node.create_subscription(PoseStamped, f'/vr/raw/{side}/wrist_pose',
            lambda msg, s=side: on_pose(msg, s), qos_profile_sensor_data)

    def spin_for(seconds):
        end = time.monotonic()+seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.03)

    def capture(side, instruction):
        while True:
            input(f'\n{instruction}\nEnter → {options.delay:g}秒後から1.2秒記録。手をキーボードから離してください: ')
            spin_for(options.delay)
            print('\a記録開始。そのまま静止してください。', flush=True)
            start = time.monotonic()
            spin_for(1.2)
            now = time.monotonic()
            history = samples[side]
            try:
                if now-frame_seen[0] > 2.5:
                    raise ValueError('world_v1の配信なし。Vuerを更新・再起動してEnter VRしてください')
                window = [m for t, _, m in history if t >= start]
                audit('capture', side=side, instruction=instruction, samples=[m.tolist() for m in window])
                times = [t for t, _, _ in history if t >= start]
                if not times or now-times[-1] > .2 or times[-1]-times[0] < .8:
                    raise ValueError('VR入力が途切れました。Questの接続を確認してください')
                if max(np.diff(times), default=0.) > .25:
                    raise ValueError('VR入力間隔が不安定です')
                result = average_pose(window)
                audit('capture_ok', side=side, instruction=instruction, pose=result.tolist())
                print('\a記録OK', flush=True)
                return result
            except ValueError as error:
                audit('capture_rejected', side=side, instruction=instruction, reason=str(error))
                print(f'再記録: {error}', flush=True)

    results, recordings = {}, {}
    try:
        print('実機への指令を出さない校正です。途中終了はCtrl+C。')
        print('QuestはEnter VR状態にしてください。ヘッドセットの画面を見る必要はありません。')
        print('あなたの前・左・上への方向だけを記録します。移動距離は揃えなくて構いません。')
        print('小さな揺れと区別できる程度に動かしてください。移動倍率は1倍のまま保存します。')
        print('回転は手首の位置を保って各45度程度。前後軸/左右軸/上下軸の3方向です。')
        print('左右とも、校正中は体の向きを変えず、VRの再センタリングをしないでください。')
        if options.resume_log:
            results, recordings = recover_recordings(options.resume_log.expanduser())
            print(f'記録を再計算して復元: {", ".join(results) or "なし"}')
            print('復元には同じVRセッション・同じ装着位置が必要です。')
        for side, label in (('left', '左手'), ('right', '右手')):
            if side in results:
                print(f'{label}は記録から復元済み。次の手へ進みます。')
                continue
            while True:
                translations, rotations = [], []
                print(f'\n===== {label} =====')
                for direction in ('前', '左', '上'):
                    while True:
                        neutral = capture(side, f'{label}・{direction}への移動: 基準姿勢に戻してください。手のひらは実機と同じ向き。')
                        endpoint = capture(side, f'{label}: 向きを変えず、基準から{direction}へ動かしてください。距離は自由です。')
                        try:
                            validate_translation(neutral, endpoint)
                            translations.append((neutral, endpoint))
                            break
                        except ValueError as error:
                            audit('translation_rejected', side=side, direction=direction, reason=str(error),
                                  initial=neutral.tolist(), endpoint=endpoint.tolist())
                            print(f'{label}・{direction}だけ再記録: {error}', flush=True)
                for direction in ('前後軸まわり（ロール）', '左右軸まわり（ピッチ）', '上下軸まわり（ヨー）'):
                    neutral = capture(side, f'{label}: 基準姿勢に戻してください。')
                    endpoint = capture(side, f'{label}: 手首中心を動かさず、{direction}に約45度回転。')
                    rotations.append((neutral, endpoint))
                try:
                    fit = fit_motion(translations, rotations)
                    results[side] = fit
                    recordings[side] = {'translations': [[a.tolist(), b.tolist()] for a, b in translations],
                                        'rotations': [[a.tolist(), b.tolist()] for a, b in rotations]}
                    audit('fit_ok', side=side, **recordings[side], result=fit)
                    residual = fit['pivot_residual_m']
                    residual_text = '推定不可' if residual is None else f'{residual*1000:.1f}mm'
                    print(f'{label} OK: 軸補正RPY={np.round(fit["axes_rpy_deg"], 1)}, '
                          f'倍率={fit["position_scale"]:.3f}, 手首推定残差={residual_text}')
                    for warning in fit['warnings']:
                        print(f'注意: {warning}', flush=True)
                    break
                except ValueError as error:
                    audit('fit_rejected', side=side, reason=str(error),
                          translations=[[a.tolist(), b.tolist()] for a, b in translations],
                          rotations=[[a.tolist(), b.tolist()] for a, b in rotations])
                    print(f'{label}の校正不成立: {error}\nこの手の6方向をやり直します。')
        params = dict(motion_reference='world_v1')
        for side, fit in results.items():
            for suffix in ('axes_rpy_deg', 'position_scale', 'mount_xyz'):
                params[f'{side}_{suffix}'] = fit[suffix]
        # Save evidence before replacing the active calibration; old calibration
        # survives cancellation or invalid data at any earlier stage.
        stamp = time.strftime('%Y%m%d-%H%M%S')
        report = options.output.with_name(f'vr-motion-recording-{stamp}.json')
        report.parent.mkdir(parents=True, exist_ok=True)
        report.write_text(json.dumps(dict(results=results, recordings=recordings), indent=2))
        atomic_yaml(options.output, {'sharpa_vr_bridge': {'ros__parameters': params}})
        print(f'\n保存完了: {options.output}\n計測記録: {report}\n追従は停止したままです。次は ./preview_vr.sh でモデルを確認できます。')
    except (KeyboardInterrupt, EOFError):
        audit('cancelled')
        print('\n校正中止。既存の校正ファイルは変更しません。')
        return 130
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0
