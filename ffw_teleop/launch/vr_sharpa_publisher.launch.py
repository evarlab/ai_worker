"""Official SG2 controller tracking, with all non-arm outputs isolated."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    remaps = [
        ('/l_wrist_pose', '/vr/raw/left/wrist_pose'),
        ('/r_wrist_pose', '/vr/raw/right/wrist_pose'),
        ('/vr_controller/left_squeeze', '/vr/input/left/squeeze'),
        ('/vr_controller/right_squeeze', '/vr/input/right/squeeze'),
        ('/leader/joystick_controller_left/joint_trajectory', '/vr/unused/head'),
        ('/leader/joystick_controller_right/joint_trajectory', '/vr/unused/lift'),
        ('/leader/joint_trajectory_command_broadcaster_left/raw_joint_trajectory', '/vr/unused/left/gripper'),
        ('/leader/joint_trajectory_command_broadcaster_right/raw_joint_trajectory', '/vr/unused/right/gripper'),
        ('/cmd_vel', '/vr/unused/cmd_vel'),
        ('/reactivate', '/vr/input/activate'),
    ]
    return LaunchDescription([
        DeclareLaunchArgument('squeeze_threshold', default_value='0.0'),
        Node(package='ffw_teleop', executable='sharpa_vuer_publisher', output='screen',
             parameters=[{'reactivate_topic': '/vr/input/activate',
                          'goal_pose_squeeze_threshold': ParameterValue(
                              LaunchConfiguration('squeeze_threshold'), value_type=float),
                          'goal_pose_position_scale': 1.0,
                          'controller_back_offset_m': 0.0,
                          'apply_lift_to_arm_z': False}],
             remappings=remaps),
    ])
