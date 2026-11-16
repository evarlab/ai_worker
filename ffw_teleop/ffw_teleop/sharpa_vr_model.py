"""Prepare the actual Sharpa URDF for the scalar-joint Cyclo VR solver."""
from itertools import combinations
from functools import lru_cache
import math
from pathlib import Path
import struct
from urllib.parse import unquote, urlparse
import xml.etree.ElementTree as ET


@lru_cache(maxsize=256)
def stl_bounds(filename):
    data = Path(filename).read_bytes()
    count = struct.unpack_from('<I', data, 80)[0] if len(data) >= 84 else 0
    low, high = [math.inf]*3, [-math.inf]*3
    if count and 84 + 50*count <= len(data):
        vertices = (struct.unpack_from('<3f', data, 84 + 50*i + 12 + 12*j)
                    for i in range(count) for j in range(3))
    else:
        vertices = (tuple(map(float, line.split()[1:]))
                    for line in data.decode('ascii').splitlines()
                    if line.strip().lower().startswith('vertex '))
    for vertex in vertices:
        if len(vertex) != 3 or not all(math.isfinite(x) for x in vertex):
            raise ValueError(f'Invalid STL vertex: {filename}')
        for i, value in enumerate(vertex):
            low[i], high[i] = min(low[i], value), max(high[i], value)
    if not all(math.isfinite(x) for x in low+high):
        raise ValueError(f'Empty STL: {filename}')
    return low, high


def simplify_collision_meshes(robot):
    """Use enclosing per-link STL boxes; retain all cross-hand/body collision pairs."""
    for collision in robot.findall('./link/collision'):
        geometry = collision.find('geometry')
        mesh = geometry.find('mesh') if geometry is not None else None
        if mesh is None:
            continue
        filename = mesh.get('filename')
        if Path(filename).suffix.lower() != '.stl':
            raise ValueError(f'Collision proxy needs an STL mesh: {filename}')
        low, high = stl_bounds(filename)
        scale = list(map(float, mesh.get('scale', '1 1 1').split()))
        if len(scale) != 3 or not all(math.isfinite(x) and x != 0 for x in scale):
            raise ValueError('Invalid mesh scale')
        center = [(a+b)*s/2 for a, b, s in zip(low, high, scale)]
        size = [max(1e-5, abs((b-a)*s)) for a, b, s in zip(low, high, scale)]
        origin = collision.find('origin')
        if origin is None:
            origin = ET.SubElement(collision, 'origin')
        xyz = list(map(float, origin.get('xyz', '0 0 0').split()))
        roll, pitch, yaw = map(float, origin.get('rpy', '0 0 0').split())
        cr, sr, cp, sp, cy, sy = (math.cos(roll), math.sin(roll), math.cos(pitch),
                                math.sin(pitch), math.cos(yaw), math.sin(yaw))
        rotation = ((cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr),
                    (sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr), (-sp, cp*sr, cp*cr))
        shifted = [xyz[i] + sum(rotation[i][j]*center[j] for j in range(3)) for i in range(3)]
        origin.set('xyz', ' '.join(map(str, shifted)))
        geometry.remove(mesh)
        ET.SubElement(geometry, 'box', size=' '.join(map(str, size)))


def prepare_model(urdf_xml, srdf_xml, resolve_package):
    robot = ET.fromstring(urdf_xml)
    links = {link.get('name') for link in robot.findall('link')}
    arms = {f'arm_{side}_joint{i}' for side in ('l', 'r') for i in range(1, 8)}
    found = set()
    children = {}
    disabled = set()
    for joint in robot.findall('joint'):
        parent, child = joint.find('parent').get('link'), joint.find('child').get('link')
        children.setdefault(parent, []).append(child)
        disabled.add(tuple(sorted((parent, child))))
        name, kind = joint.get('name'), joint.get('type')
        if name in arms:
            found.add(name)
        elif kind != 'fixed':
            # Cyclo uses model.nq as its velocity dimension. Continuous joints
            # have nq=2/nv=1 in Pinocchio; scalar revolute joints avoid that mismatch.
            if kind == 'continuous':
                joint.set('type', 'revolute')
            limit = joint.find('limit')
            if limit is None:
                limit = ET.SubElement(joint, 'limit')
            limit.set('velocity', '0')
            limit.attrib.setdefault('effort', '1')
            if kind == 'continuous':
                limit.set('lower', '-1000000000')
                limit.set('upper', '1000000000')
    if found != arms:
        raise ValueError(f'Expected 14 SG2 arm joints; missing {sorted(arms - found)}')
    for side in ('left', 'right'):
        root = f'{side}_adapter_mount'
        if root not in links:
            raise ValueError(f'Missing mounted Sharpa hand: {root}')
        cluster, queue = set(), [root]
        while queue:
            link = queue.pop()
            if link not in cluster:
                cluster.add(link)
                queue.extend(children.get(link, []))
        # The glove owns finger motion. Internal same-hand collisions cannot be
        # resolved by an arm-only QP. Keep hand-vs-body and hand-vs-other-hand pairs.
        disabled.update(tuple(sorted(pair)) for pair in combinations(cluster, 2))
        wrist = f'arm_{"l" if side == "left" else "r"}_link7'
        for link in (root, f'{side}_hand_wrist', f'{side}_hand_C_MC'):
            if link in links:
                disabled.add(tuple(sorted((wrist, link))))
    if srdf_xml:
        for pair in ET.fromstring(srdf_xml).findall('disable_collisions'):
            a, b = pair.get('link1'), pair.get('link2')
            if a in links and b in links:
                disabled.add(tuple(sorted((a, b))))
    # Pinocchio/coal must resolve meshes even without an RViz resource retriever.
    for mesh in robot.iter('mesh'):
        filename = mesh.get('filename', '')
        if filename.startswith('package://'):
            package, path = filename[len('package://'):].split('/', 1)
            full = Path(resolve_package(package)) / path
            if not full.is_file():
                raise FileNotFoundError(full)
            mesh.set('filename', str(full))
        elif filename.startswith('file:'):
            full = Path(unquote(urlparse(filename).path))
            if not full.is_file():
                raise FileNotFoundError(full)
            mesh.set('filename', str(full))
    simplify_collision_meshes(robot)
    srdf = ET.Element('robot', name=robot.get('name', 'sharpa_vr'))
    for a, b in sorted(disabled):
        ET.SubElement(srdf, 'disable_collisions', link1=a, link2=b, reason='AdjacentOrSameHand')
    return ET.tostring(robot, encoding='unicode'), ET.tostring(srdf, encoding='unicode')
