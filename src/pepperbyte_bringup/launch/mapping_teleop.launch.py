# Copyright (c) 2026 Peppermint Robotics. All rights reserved.
#
# Mode 1: Teleop mapping with gamepad
# Launches: cobra_driver, description, SLAM, joy + teleop_twist_joy, twist_mux
# Does NOT launch Nav2.
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    bringup_share = get_package_share_directory('pepperbyte_bringup')
    joy_config = os.path.join(bringup_share, 'config', 'joy_params.yaml')
    teleop_config = os.path.join(bringup_share, 'config', 'teleop_twist_joy_params.yaml')
    twist_mux_config = os.path.join(bringup_share, 'config', 'twist_mux.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true'
        ),

        # --- Robot description (URDF + robot_state_publisher) ---
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('pepperbyte_description'),
                    'launch', 'description.launch.py'
                )
            ),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),

        # --- Cobra driver (serial bridge) ---
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('cobra_driver'),
                    'launch', 'cobra_driver.launch.py'
                )
            ),
        ),

        # --- SLAM ---
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('pepperbyte_slam'),
                    'launch', 'slam.launch.py'
                )
            ),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),

        # --- Joystick driver ---
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[joy_config],
        ),

        # --- Teleop twist joy (output remapped to joy_vel for twist_mux) ---
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy_node',
            output='screen',
            parameters=[teleop_config],
            remappings=[('cmd_vel', 'joy_vel')],
        ),

        # --- Twist mux (merges joy_vel → /cmd_vel) ---
        Node(
            package='twist_mux',
            executable='twist_mux',
            name='twist_mux',
            output='screen',
            parameters=[twist_mux_config],
            remappings=[('cmd_vel_out', '/cmd_vel')],
        ),
    ])
