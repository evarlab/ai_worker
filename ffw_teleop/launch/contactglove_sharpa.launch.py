"""Run the Sharpa bridge separately from the user-controlled glove source."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=PathJoinSubstitution([
            FindPackageShare('ffw_teleop'), 'config', 'contactglove_sharpa_ros.yaml'])),
        Node(package='ffw_teleop', executable='contactglove_sharpa', output='screen',
             parameters=[LaunchConfiguration('params_file')]),
    ])
