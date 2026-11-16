"""Portable URDF visuals for rosbag replay without robot-side mesh paths."""
from pathlib import Path
from urllib.parse import unquote, urlparse
import xml.etree.ElementTree as ET


def bundle_model(xml, directory, resolve_package):
    directory = Path(directory)
    (directory/'meshes').mkdir(parents=True, exist_ok=True)
    robot = ET.fromstring(xml)
    meshes = {}
    for mesh in robot.findall('.//mesh'):
        uri = mesh.get('filename')
        if uri.startswith('package://'):
            package, relative = uri[len('package://'):].split('/', 1)
            path = Path(resolve_package(package))/unquote(relative)
        elif uri.startswith('file://'):
            path = Path(unquote(urlparse(uri).path))
        else:
            path = Path(uri)
        path = path.resolve(strict=True)
        if path not in meshes:
            name = f'{len(meshes):03d}_{path.name}'
            (directory/'meshes'/name).write_bytes(path.read_bytes())
            meshes[path] = name
        mesh.set('filename', '__BAG_MODEL_ROOT__/meshes/'+meshes[path])
    for tag in ('ros2_control', 'gazebo'):
        for element in robot.findall(tag):
            robot.remove(element)
    (directory/'robot.urdf').write_text(ET.tostring(robot, encoding='unicode'))


def replay_model(directory):
    directory = Path(directory).resolve()
    source = directory/'robot.urdf'
    target = directory/'robot.replay.urdf'
    target.write_text(source.read_text().replace('__BAG_MODEL_ROOT__', 'file://'+str(directory)))
    return target
