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
