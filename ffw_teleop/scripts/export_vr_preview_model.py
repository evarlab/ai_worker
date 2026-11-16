#!/usr/bin/env python3
"""Bundle the installed real visual meshes and Sharpa URDF for local RViz."""
from pathlib import Path
import subprocess
import tarfile
import tempfile
from urllib.parse import unquote, urlparse
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory


def main():
    description = Path(get_package_share_directory('ffw_description'))
    xml = subprocess.check_output(['xacro', str(description / 'urdf/ffw_sg2_rev1_follower/ffw_sg2_follower.urdf.xacro'),
        'end_effector:=sharpa', 'hands:=both', 'use_mock_hardware:=true', 'model:=ffw_sg2_rev1_follower'], text=True)
    robot = ET.fromstring(xml)
    root = Path(tempfile.mkdtemp(prefix='vr-visual-model-'))
    (root/'meshes').mkdir()
    meshes = {}
    for mesh in robot.findall('.//mesh'):
        uri = mesh.get('filename')
        if uri.startswith('package://'):
            package, relative = uri[len('package://'):].split('/', 1)
            path = Path(get_package_share_directory(package))/unquote(relative)
        elif uri.startswith('file://'):
            path = Path(unquote(urlparse(uri).path))
        else:
            path = Path(uri)
        path = path.resolve(strict=True)
        if path not in meshes:
            filename = f'{len(meshes):03d}_{path.name}'
            (root/'meshes'/filename).write_bytes(path.read_bytes())
            meshes[path] = filename
        mesh.set('filename', '__PREVIEW_MESH_ROOT__/meshes/'+meshes[path])
    for tag in ('ros2_control', 'gazebo'):
        for element in robot.findall(tag):
            robot.remove(element)
    (root/'robot.urdf').write_text(ET.tostring(robot, encoding='unicode'))
    target = Path('/tmp/vr-preview-model.tar.gz')
    with tarfile.open(target, 'w:gz') as archive:
        archive.add(root/'robot.urdf', arcname='robot.urdf')
        archive.add(root/'meshes', arcname='meshes')
    print(target)


if __name__ == '__main__':
    main()
