"""One shared arm reference for the model and physical startup."""
import math


def relaxed_arm_positions(palms='inward'):
    positions = {f'arm_{side}_joint{i}': 0. for side in ('l', 'r') for i in range(1, 8)}
    if palms == 'inward':
        positions['arm_l_joint5'] = -math.pi/2
        positions['arm_r_joint5'] = math.pi/2
    elif palms != 'front':
        raise ValueError('palms must be inward or front')
    return positions


def load_start_pose(path):
    """Load a reviewed 14-joint arm target; a mismatched lift must not auto-move."""
    import json
    from pathlib import Path
    profile = json.loads(Path(path).read_text())
    expected = {f'arm_{side}_joint{i}' for side in ('l', 'r') for i in range(1, 8)}
    positions = profile['arm_positions']
    if set(positions) != expected or not all(isinstance(v, (float, int)) and not isinstance(v, bool) and math.isfinite(v) for v in positions.values()):
        raise ValueError('待機姿勢には有限な両腕14関節の値が必要です')
    limits = profile['joint_limits']
    if set(limits) != expected:
        raise ValueError('待機姿勢の関節制限が不足しています')
    for name, value in positions.items():
        low, high = limits[name]
        if not math.isfinite(low) or not math.isfinite(high) or not low <= value <= high:
            raise ValueError('待機姿勢が関節制限の範囲外です: '+name)
    lift = profile['reference_lift_m']
    tolerance = profile.get('lift_tolerance_m', .01)
    if not math.isfinite(lift) or not -.5 <= lift <= 0 or not math.isfinite(tolerance) or not 0 < tolerance <= .02:
        raise ValueError('待機姿勢のlift基準が不正です')
    return dict(positions), float(lift), float(tolerance)
