"""Live-URDF frames for human-demo replay; metres and radians throughout."""
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation
from .human_demo_data import rigid


class RobotFrames:
    def __init__(self, xml):
        self.root = ET.fromstring(xml)
        self.joints = {}
        children = set()
        parents = set()
        for j in self.root.findall('joint'):
            parent, child = j.find('parent').get('link'), j.find('child').get('link')
            origin = j.find('origin')
            t = np.eye(4)
            if origin is not None:
                t[:3, 3] = np.fromstring(origin.get('xyz', '0 0 0'), sep=' ')
                t[:3, :3] = Rotation.from_euler('xyz', np.fromstring(origin.get('rpy', '0 0 0'), sep=' ')).as_matrix()
            axis = j.find('axis')
            v = np.fromstring(axis.get('xyz', '1 0 0'), sep=' ') if axis is not None else np.array([1., 0., 0.])
            self.joints[j.get('name')] = (parent, child, rigid(t), v, j.get('type'))
            children.add(child); parents.add(parent)
        roots = parents - children
        if len(roots) != 1 or roots != {'base_link'}:
            raise ValueError('Expected a single base_link URDF root')

    def transforms(self, values):
        result = {'base_link': np.eye(4)}
        remaining = dict(self.joints)
        while remaining:
            done = []
            for name, (parent, child, origin, axis, kind) in remaining.items():
                if parent not in result:
                    continue
                move = np.eye(4)
                if kind != 'fixed':
                    if name not in values or not np.isfinite(values[name]):
                        raise ValueError('Missing or invalid joint feedback: ' + name)
                    if kind in ('revolute', 'continuous'):
                        move[:3, :3] = Rotation.from_rotvec(axis * values[name]).as_matrix()
                    elif kind == 'prismatic':
                        move[:3, 3] = axis * values[name]
                    else:
                        raise ValueError('Unsupported joint: ' + name)
                result[child] = result[parent] @ origin @ move
                done.append(name)
            if not done:
                raise ValueError('Disconnected or cyclic URDF')
            for name in done:
                del remaining[name]
        return result

    def fixed_transform(self, parent, child):
        by_child = {j[1]: j for j in self.joints.values()}
        result = np.eye(4)
        while child != parent:
            p, _, origin, _, kind = by_child[child]
            if kind != 'fixed':
                raise ValueError('Mount chain is not fixed')
            result = origin @ result
            child = p
        return rigid(result)


def load_live_robot_scene(xml, workspace=None):
    """Use live arm/head/mount frames with the existing collision meshes.

    The imported simulator's +150 mm root placement and +25.4 mm torso X
    offset must not enter real base_link targets. Meshes remain approximations.
    """
    from ffw_sharpa_sim import model as source
    from .human_demo_scene import load_scene, WORKSPACE
    frames = RobotFrames(xml)
    mounts = [frames.fixed_transform('arm_' + s + '_link7', name + '_hand_C_MC')
              for s, name in (('l', 'left'), ('r', 'right'))]
    if not np.allclose(mounts[0], mounts[1], atol=1e-8):
        raise ValueError('Asymmetric mounts require separate model attachments')
    original_build, original_pos, original_quat = source.build_spec, source.HAND_POS, source.HAND_QUAT
    def build():
        spec = original_build()
        spec.body('base_link').pos = [0, 0, 0]
        for name in ('lift_joint', 'head_joint1', 'head_joint2'):
            _, child, origin, axis, _ = frames.joints[name]
            body = spec.body(child)
            body.pos = origin[:3, 3]
            body.quat = Rotation.from_matrix(origin[:3, :3]).as_quat()[[3, 0, 1, 2]]
            spec.joint(name).axis = axis
        return spec
    try:
        source.HAND_POS = tuple(mounts[0][:3, 3])
        q = Rotation.from_matrix(mounts[0][:3, :3]).as_quat()
        source.HAND_QUAT = tuple(q[[3, 0, 1, 2]])
        source.build_spec = build
        return load_scene(workspace or WORKSPACE)
    finally:
        source.build_spec, source.HAND_POS, source.HAND_QUAT = original_build, original_pos, original_quat


def table_relative_targets(board_hands, base_board):
    """A shared rigid board transform preserves hand spacing, orientation and height."""
    poses = np.asarray(board_hands, dtype=float)
    if poses.shape[-2:] != (4, 4):
        raise ValueError('Expected hand poses')
    return rigid(base_board) @ poses


def recorded_finger_targets(values, config, side, current, limits):
    """Use the same all/four_fingers selector as the live glove bridge.

    Unmapped hand joints retain measured positions. Apply configured glove and
    URDF joint limits; collision validation is still required by the caller.
    """
    from .contactglove_mapping import FLEXION, ABDUCTION, targets, abduction_targets
    selection = config.get('controlled_fingers', 'all')
    if selection not in ('all', 'four_fingers'):
        raise ValueError('controlled_fingers must be all or four_fingers')
    if side not in ('left', 'right'):
        raise ValueError('Invalid hand side')
    result = dict(current)
    flexion = targets(values, config[side+'_gains'], config[side+'_offsets'],
                      config[side+'_lower'], config[side+'_upper'])
    mapping = list(zip(FLEXION, flexion))
    if config.get('include_abduction', False):
        mapping += list(zip(ABDUCTION, abduction_targets(values, config[side+'_abduction_gains'], config[side+'_abduction_offsets'])))
    for (joint, _, _), value in mapping:
        if selection == 'four_fingers' and joint.startswith('thumb'):
            continue
        name = side+'_'+joint
        if name not in result or name not in limits:
            raise ValueError('Missing live hand joint: '+name)
        low, high = limits[name]
        if not np.isfinite([value, low, high]).all() or low > high:
            raise ValueError('Invalid hand target/limits: '+name)
        result[name] = float(np.clip(value, low, high))
    return result


def track_fixed_board(observation, anchor, ros_time_ns, previous=None, max_age_s=.5):
    """Monitor a fixed, previously calibrated board without re-anchoring it.

    A fresh single-frame pose may confirm the existing anchor while the
    observer rebuilds its averaging window. Missing frames can only reuse a
    verified sample until its original camera timestamp expires.
    """
    if observation.get('valid'):
        sample = dict(T_base_board=observation['T_base_board'], image_stamp_ns=observation['image_stamp_ns'])
    elif 'T_base_board_candidate' in observation:
        if not observation.get('reason', '').startswith('Waiting for stable board samples'):
            raise ValueError('Board observation is unstable')
        sample = dict(T_base_board=observation['T_base_board_candidate'], image_stamp_ns=observation['image_stamp_ns'])
    else:
        if observation.get('reason') in ('Observer stopped', 'Observer starting'):
            raise ValueError('Board observer is stopped')
        if previous is None:
            raise ValueError('No verified board observation')
        sample = previous
    age = (ros_time_ns-sample['image_stamp_ns'])/1e9
    if not -.1 <= age <= max_age_s:
        raise ValueError('Board camera observation is stale')
    current = rigid(sample['T_base_board']); fixed = rigid(anchor)
    if np.linalg.norm(current[:3,3]-fixed[:3,3]) > .01 or Rotation.from_matrix(fixed[:3,:3].T@current[:3,:3]).magnitude() > np.deg2rad(2):
        raise ValueError('Board moved')
    return sample
