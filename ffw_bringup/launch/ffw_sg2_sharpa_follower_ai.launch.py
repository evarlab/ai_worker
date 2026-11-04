#!/usr/bin/env python3
#
# Copyright 2026 evarlab
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Based on ffw_sg2_follower_ai.launch.py (ROBOTIS).

"""
SG2 follower with Sharpa Wave hands (or bare flanges) instead of the grippers.

One controller manager runs the SG2 body (Dynamixel, grippers removed) and each mounted hand
(ffw_sharpa_hardware), so /joint_states carries body and finger joints together and every part is
commanded through its trajectory controller:

    arm_l_controller, arm_r_controller   7 joints each
    hand_l_controller, hand_r_controller 22 joints each (only for mounted hands)
    head_controller, lift_controller, swerve_drive_controller

hands:=none brings up the body without grippers or hands. The default model is the built robot:
A v2 adapters, both hands at 0 deg about the wrist axis.

Hand identities are never stored in the repository: pass them as arguments or set
SHARPA_LEFT_IP / SHARPA_RIGHT_IP / SHARPA_LEFT_SERIAL / SHARPA_RIGHT_SERIAL (empty: the factory IP
of that side). A serial number, when given, takes precedence over the IP.
"""

from ffw_bringup.launch_utils import command_topic
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import LogInfo
from launch.actions import OpaqueFunction
from launch.actions import RegisterEventHandler
from launch.actions import TimerAction
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch.substitutions import EnvironmentVariable
from launch.substitutions import FindExecutable
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

HANDS = {'both': ('left', 'right'), 'left': ('left',), 'right': ('right',), 'none': ()}
HAND_CONTROLLERS = {'left': 'hand_l', 'right': 'hand_r'}


def declare_arguments():
    return [
        DeclareLaunchArgument('hands', default_value='both', choices=list(HANDS),
                              description='Mounted Sharpa hands; none: bare flanges.'),
        DeclareLaunchArgument('sharpa_adapter', default_value='a_v2', choices=['a_v2'],
                              description='Wrist adapter (A v2: arms keep 7 joints).'),
        DeclareLaunchArgument('left_hand_mount_deg', default_value='0',
                              description='Left hand rotation about the wrist axis [deg].'),
        DeclareLaunchArgument('right_hand_mount_deg', default_value='0',
                              description='Right hand rotation about the wrist axis [deg].'),
        DeclareLaunchArgument('left_hand_ip',
                              default_value=EnvironmentVariable('SHARPA_LEFT_IP',
                                                                default_value=''),
                              description='Left hand IP (empty: factory default).'),
        DeclareLaunchArgument('right_hand_ip',
                              default_value=EnvironmentVariable('SHARPA_RIGHT_IP',
                                                                default_value=''),
                              description='Right hand IP (empty: factory default).'),
        DeclareLaunchArgument('left_hand_serial',
                              default_value=EnvironmentVariable('SHARPA_LEFT_SERIAL',
                                                                default_value=''),
                              description='Left hand serial number (overrides the IP).'),
        DeclareLaunchArgument('right_hand_serial',
                              default_value=EnvironmentVariable('SHARPA_RIGHT_SERIAL',
                                                                default_value=''),
                              description='Right hand serial number (overrides the IP).'),
        DeclareLaunchArgument('hand_speed_coeff',
                              default_value=EnvironmentVariable('SHARPA_SPEED_COEFF',
                                                                default_value='0.3'),
                              description='Sharpa joint speed coefficient, (0, 0.5].'),
        DeclareLaunchArgument('start_rviz', default_value='false',
                              description='Whether to execute rviz2'),
        DeclareLaunchArgument('use_mock_hardware', default_value='false',
                              description='Use mock hardware mirroring command (body and hands).'),
        DeclareLaunchArgument('mock_sensor_commands', default_value='false',
                              description='Enable mock sensor commands.'),
        DeclareLaunchArgument('launch_cameras', default_value='true',
                              description='Whether to launch cameras.'),
        DeclareLaunchArgument('head_camera_type', default_value='zed',
                              choices=['zed', 'realsense'],
                              description='Head camera type.'),
        DeclareLaunchArgument('auto_assign_cameras', default_value='false',
                              choices=['true', 'false'],
                              description='Use the manual camera YAML or automatic assignment.'),
        DeclareLaunchArgument('launch_lidar', default_value='true',
                              description='Whether to launch lidar.'),
        DeclareLaunchArgument('init_position', default_value='true',
                              description='Move arms, head, lift and steering to zero and open '
                                          'the hands at start-up.'),
        DeclareLaunchArgument('model', default_value='ffw_sg2_rev1_follower',
                              description='Robot model name.'),
        DeclareLaunchArgument('use_head_eef_tracker', default_value='false',
                              description='Whether to launch the head EEF tracker node.'),
        DeclareLaunchArgument('init_position_file',
                              default_value='ffw_sg2_sharpa_follower_initial_positions.yaml',
                              description='Initial position file.'),
    ]


def launch_setup(context):
    hands = HANDS[LaunchConfiguration('hands').perform(context)]
    start_rviz = LaunchConfiguration('start_rviz')
    use_mock_hardware = LaunchConfiguration('use_mock_hardware')
    launch_cameras = LaunchConfiguration('launch_cameras')
    launch_lidar = LaunchConfiguration('launch_lidar')
    init_position = LaunchConfiguration('init_position')
    model = LaunchConfiguration('model')
    use_head_eef_tracker = LaunchConfiguration('use_head_eef_tracker')

    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]), ' ',
        PathJoinSubstitution([FindPackageShare('ffw_description'), 'urdf', model,
                              'ffw_sg2_follower.urdf.xacro']),
        ' use_sim:=false',
        ' use_mock_hardware:=', use_mock_hardware,
        ' mock_sensor_commands:=', LaunchConfiguration('mock_sensor_commands'),
        ' model:=', model,
        ' end_effector:=', 'sharpa' if hands else 'none',
        ' hands:=', LaunchConfiguration('hands') if hands else 'both',
        ' left_hand_ip:="', LaunchConfiguration('left_hand_ip'), '"',
        ' right_hand_ip:="', LaunchConfiguration('right_hand_ip'), '"',
        ' left_hand_serial:="', LaunchConfiguration('left_hand_serial'), '"',
        ' right_hand_serial:="', LaunchConfiguration('right_hand_serial'), '"',
        ' hand_speed_coeff:=', LaunchConfiguration('hand_speed_coeff'),
        ' sharpa_adapter:=', LaunchConfiguration('sharpa_adapter'),
        ' left_hand_mount_deg:=', LaunchConfiguration('left_hand_mount_deg'),
        ' right_hand_mount_deg:=', LaunchConfiguration('right_hand_mount_deg'),
    ])
    robot_description = {
        'robot_description': ParameterValue(robot_description_content, value_type=str)}

    config_dir = PathJoinSubstitution([FindPackageShare('ffw_bringup'), 'config', model])
    controller_manager_config = PathJoinSubstitution(
        [config_dir, 'ffw_sg2_sharpa_follower_ai_hardware_controller.yaml'])
    trajectory_params_file = PathJoinSubstitution(
        [config_dir, LaunchConfiguration('init_position_file')])
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare('ffw_description'), 'rviz', 'ffw_sg2.rviz'])

    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, controller_manager_config],
        output='both',
    )
    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description],
        output='screen',
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen',
        condition=IfCondition(start_rviz),
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen',
    )
    # Leader head/lift joysticks keep working; the leader's arm topics carry gripper joints, so
    # the arm controllers keep their own topics (/arm_?_controller/joint_trajectory).
    remaps = []
    for controller, group, legacy in (
            ('head', 'head', '/leader/joystick_controller_left/joint_trajectory'),
            ('lift', 'lift', '/leader/joystick_controller_right/joint_trajectory')):
        remaps += ['--controller-ros-args',
                   f'-r /{controller}_controller/joint_trajectory:='
                   f'{command_topic(group, legacy)}']
    body_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=remaps + ['arm_l_controller', 'arm_r_controller', 'head_controller',
                            'lift_controller', 'ffw_robot_manager'],
        output='screen',
    )
    # Hands get their own spawner: a hand that is off or faulted fails only this one.
    hand_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[f'{HAND_CONTROLLERS[side]}_controller' for side in hands],
        output='screen',
    ) if hands else None

    swerve_steering_initial_position_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['swerve_steering_initial_position_controller'],
        condition=IfCondition(init_position),
    )
    swerve_steering_initial_position_unspawner = Node(
        package='controller_manager',
        executable='unspawner',
        arguments=['swerve_steering_initial_position_controller'],
        output='screen',
    )
    swerve_drive_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['swerve_drive_controller'],
        output='screen',
    )
    swerve_drive_spawner_direct = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['swerve_drive_controller'],
        output='screen',
        condition=UnlessCondition(init_position),
    )

    def executor(name):
        return Node(
            package='ffw_bringup',
            executable='joint_trajectory_executor',
            name=f'{name}_joint_trajectory_executor',
            parameters=[trajectory_params_file],
            output='screen',
        )

    body_executors = [executor(name) for name in ('arm_l', 'arm_r', 'head', 'lift')]
    hand_executors = [executor(HAND_CONTROLLERS[side]) for side in hands]
    steering_executor = Node(
        package='ffw_bringup',
        executable='joint_trajectory_executor',
        name='swerve_steering_joint_trajectory_executor',
        parameters=[trajectory_params_file],
        output='screen',
        condition=IfCondition(init_position),
    )

    actions = [
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        RegisterEventHandler(OnProcessExit(target_action=joint_state_broadcaster_spawner,
                                           on_exit=[rviz_node])),
        body_controller_spawner,
        swerve_steering_initial_position_spawner,
        swerve_drive_spawner_direct,
        RegisterEventHandler(
            OnProcessExit(target_action=body_controller_spawner,
                          on_exit=body_executors + [steering_executor]),
            condition=IfCondition(init_position)),
        RegisterEventHandler(
            OnProcessExit(target_action=steering_executor,
                          on_exit=[swerve_steering_initial_position_unspawner,
                                   TimerAction(period=5.0, actions=[swerve_drive_spawner])]),
            condition=IfCondition(init_position)),
    ]
    if hand_controller_spawner is not None:
        actions += [
            hand_controller_spawner,
            RegisterEventHandler(
                OnProcessExit(target_action=hand_controller_spawner,
                              on_exit=lambda event, _context: hand_executors
                              if event.returncode == 0 else
                              [LogInfo(msg='Hand controllers failed; hands not moved.')]),
                condition=IfCondition(init_position)),
        ]

    bringup_launch_dir = PathJoinSubstitution([FindPackageShare('ffw_bringup'), 'launch'])
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([bringup_launch_dir,
                                                            'camera.launch.py'])),
        launch_arguments={
            'head_camera_type': LaunchConfiguration('head_camera_type'),
            'auto_assign_cameras': LaunchConfiguration('auto_assign_cameras'),
        }.items(),
        condition=IfCondition(launch_cameras),
    )
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([bringup_launch_dir,
                                                            'lidar_dual.launch.py'])),
        condition=IfCondition(launch_lidar),
    )
    actions += [
        TimerAction(period=20.0, actions=[camera_launch], condition=IfCondition(init_position)),
        TimerAction(period=10.0, actions=[camera_launch],
                    condition=UnlessCondition(init_position)),
        TimerAction(period=20.0, actions=[lidar_launch], condition=IfCondition(init_position)),
        TimerAction(period=10.0, actions=[lidar_launch], condition=UnlessCondition(init_position)),
        Node(
            package='ffw_bringup',
            executable='head_eef_tracker',
            name='head_eef_tracker',
            output='screen',
            condition=IfCondition(use_head_eef_tracker),
            parameters=[{'joint_trajectory_topic': command_topic(
                'head', '/leader/joystick_controller_left/joint_trajectory')}],
        ),
        Node(
            package='dual_laser_merger',
            executable='dual_laser_merger_node',
            output='screen',
            condition=IfCondition(launch_lidar),
            parameters=[{
                'laser_1_topic': '/scan_left',
                'laser_2_topic': '/scan_right',
                'merged_scan_topic': '/scan',
                'merged_cloud_topic': '/scan_cloud',
                'target_frame': 'base_link',
                'angle_min': -3.141592654,
                'angle_max': 3.141592654,
                'angle_increment': 0.006544985,
                'scan_time': 0.1,
                'range_min': 0.05,
                'range_max': 20.0,
                'use_inf': True,
                'tolerance': 0.05,
                'queue_size': 10,
                'enable_shadow_filter': True,
                'enable_average_filter': True,
            }],
        ),
    ]
    return actions


def generate_launch_description():
    return LaunchDescription(declare_arguments() + [OpaqueFunction(function=launch_setup)])
