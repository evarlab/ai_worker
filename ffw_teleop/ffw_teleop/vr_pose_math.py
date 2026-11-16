"""Rigid controller mount and relative wrist reference transforms."""
import numpy as np
from scipy.spatial.transform import Rotation


def pose_matrix(pose):
    p = pose.position
    q = pose.orientation
    xyz = np.array([p.x, p.y, p.z], dtype=float)
    quat = np.array([q.x, q.y, q.z, q.w], dtype=float)
    if not np.isfinite(xyz).all() or not np.isfinite(quat).all() or np.linalg.norm(quat) < 1e-6:
        raise ValueError('Nonfinite pose or invalid quaternion')
    matrix = np.eye(4)
    matrix[:3, :3] = Rotation.from_quat(quat).as_matrix()
    matrix[:3, 3] = xyz
    return matrix


def mount_matrix(xyz, rpy_deg):
    if len(xyz) != 3 or len(rpy_deg) != 3 or not np.isfinite([xyz, rpy_deg]).all():
        raise ValueError('Mount requires 3 finite XYZ metres and RPY degrees')
    matrix = np.eye(4)
    matrix[:3, :3] = Rotation.from_euler('xyz', rpy_deg, degrees=True).as_matrix()
    matrix[:3, 3] = xyz
    return matrix


def relative_goal(current, initial_vr, initial_robot, scale):
    goal = np.eye(4)
    goal[:3, 3] = initial_robot[:3, 3] + scale * (current[:3, 3] - initial_vr[:3, 3])
    goal[:3, :3] = current[:3, :3] @ initial_vr[:3, :3].T @ initial_robot[:3, :3]
    return goal


def bounded_goal(current, desired, translation_step, rotation_step):
    """Advance a continuous reference without stepping past the desired pose."""
    result = current.copy()
    delta = desired[:3, 3] - current[:3, 3]
    distance = np.linalg.norm(delta)
    if distance > 0:
        result[:3, 3] += delta * min(1.0, translation_step / distance)
    rotvec = Rotation.from_matrix(desired[:3, :3] @ current[:3, :3].T).as_rotvec()
    angle = np.linalg.norm(rotvec)
    if angle > 0:
        increment = Rotation.from_rotvec(rotvec * min(1.0, rotation_step / angle)).as_matrix()
        result[:3, :3] = increment @ current[:3, :3]
    return result
