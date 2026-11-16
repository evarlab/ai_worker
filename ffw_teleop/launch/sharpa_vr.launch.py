"""Run Cyclo VR IK and an arm-only gate alongside ContactGlove finger control."""
from pathlib import Path
import subprocess
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ffw_teleop.sharpa_vr_model import prepare_model


def setup(context):
    params = LaunchConfiguration('params_file').perform(context)
    dry_run = LaunchConfiguration('dry_run').perform(context).lower() == 'true'
    description = Path(get_package_share_directory('ffw_description'))
    model_share = Path(get_package_share_directory('cyclo_motion_controller_models'))
    xacro_file = description / 'urdf/ffw_sg2_rev1_follower/ffw_sg2_follower.urdf.xacro'
    xml = subprocess.run(['xacro', str(xacro_file), 'end_effector:=sharpa', 'hands:=both',
                          'use_mock_hardware:=true', 'model:=ffw_sg2_rev1_follower'],
                         check=True, capture_output=True, text=True).stdout
    srdf = (model_share / 'models/ai_worker/ffw_sg2_follower_default.srdf').read_text()
    urdf, srdf = prepare_model(xml, srdf, get_package_share_directory)
    output = Path(tempfile.mkdtemp(prefix='sharpa-vr-'))
    (output / 'robot.urdf').write_text(urdf)
    (output / 'robot.srdf').write_text(srdf)
    divergence = [('/reference_diverged', '/vr/reference_diverged')]
    return [
        Node(package='ffw_teleop', executable='sharpa_vr_bridge', name='sharpa_vr_bridge',
             parameters=[params, {'dry_run': dry_run}], output='screen'),
        Node(package='cyclo_motion_controller_ros', executable='vr_controller_node',
             parameters=[params, {'urdf_path': str(output / 'robot.urdf'),
                                  'srdf_path': str(output / 'robot.srdf')}],
             remappings=divergence, output='screen'),
        Node(package='cyclo_motion_controller_ros', executable='reference_checker_node',
             parameters=[params], remappings=divergence, output='screen'),
    ]


def generate_launch_description():
    config = str(Path(get_package_share_directory('ffw_teleop')) / 'config/sharpa_vr.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=config),
        DeclareLaunchArgument('dry_run', default_value='true', choices=['true', 'false']),
        OpaqueFunction(function=setup),
    ])
