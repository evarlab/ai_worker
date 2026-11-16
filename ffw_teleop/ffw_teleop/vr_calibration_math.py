"""Fit motion axes and a wrist pivot from deliberately separated movements."""
import numpy as np
from scipy.spatial.transform import Rotation


def average_pose(samples):
    if len(samples) < 8:
        raise ValueError('姿勢データ不足（8サンプル以上必要）')
    poses = np.asarray(samples, dtype=float)
    if poses.shape[1:] != (4, 4) or not np.isfinite(poses).all():
        raise ValueError('不正な姿勢データ')
    xyz = poses[:, :3, 3]
    rotations = Rotation.from_matrix(poses[:, :3, :3])
    mean = rotations.mean()
    result = np.eye(4)
    result[:3, 3] = np.median(xyz, axis=0)
    result[:3, :3] = mean.as_matrix()
    return result


def validate_translation(initial, endpoint):
    delta = endpoint[:3, 3]-initial[:3, 3]
    length = float(np.linalg.norm(delta))
    if not np.isfinite(length) or length < .03:
        raise ValueError(f'移動{length*100:.1f}cm: 方向を読むため、もう少し大きく動かしてください')
    return delta


def fit_motion(translations, rotations):
    """Pairs of (neutral, endpoint), three translations and three pivot rotations.

    World->robot axes use +X forward, +Y left, +Z up. The wrist-pivot fit
    returns a local lever arm in the incoming controller orientation frame.
    """
    if len(translations) != 3 or len(rotations) != 3:
        raise ValueError('移動3方向・回転3方向が必要')
    deltas = np.column_stack([validate_translation(a, b) for a, b in translations])
    lengths = np.linalg.norm(deltas, axis=0)
    directions = deltas / lengths
    if np.max(np.abs(directions.T @ directions - np.eye(3))) > .6:
        raise ValueError('移動方向が近すぎます。前・左・上をもう少し分けてください')
    if np.linalg.det(directions) <= 0:
        raise ValueError('移動方向の符号が不一致です。右ではなく左へ動かしてください')
    u, _, vt = np.linalg.svd(directions)
    axes = (u @ vt).T
    axis_errors = np.rad2deg(np.arccos(np.clip(np.diag(axes @ directions), -1, 1)))
    # p0 + R0*r = p1 + R1*r for rotation around a fixed wrist.
    lhs, rhs, rotation_vectors, angles = [], [], [], []
    warnings = []
    if np.max(axis_errors) > 10:
        warnings.append(f'移動方向を直交座標へ補正しました。最大ずれ{np.max(axis_errors):.1f}度。URDFで方向を確認してください')
    for a, b in rotations:
        rv = Rotation.from_matrix(b[:3, :3] @ a[:3, :3].T).as_rotvec()
        angle = np.linalg.norm(rv)
        angles.append(float(np.rad2deg(angle)))
        if angle < np.deg2rad(5):
            continue
        rotation_vectors.append(rv / angle)
        lhs.append(b[:3, :3]-a[:3, :3])
        rhs.append(a[:3, 3]-b[:3, 3])
    lever, residual, pivot_applied = np.zeros(3), None, False
    if lhs:
        lhs, rhs = np.vstack(lhs), np.concatenate(rhs)
        candidate, _, rank, singular = np.linalg.lstsq(lhs, rhs, rcond=None)
        residual = float(np.sqrt(np.mean((lhs @ candidate-rhs)**2)))
        if (rank == 3 and singular[-1] > .1*singular[0]
                and np.linalg.norm(candidate) <= .35 and residual <= .030):
            lever, pivot_applied = candidate, True
    if not pivot_applied:
        warnings.append('手首中心の位置補正は推定できないため0で保存。座標軸と回転の追従は使用できます')
    return dict(axes_rpy_deg=Rotation.from_matrix(axes).as_euler('xyz', degrees=True).tolist(),
                position_scale=1.0, mount_xyz=lever.tolist(),
                translation_lengths_m=lengths.tolist(), axis_errors_deg=axis_errors.tolist(),
                pivot_residual_m=residual, pivot_applied=pivot_applied,
                rotation_angles_deg=angles, warnings=warnings)
