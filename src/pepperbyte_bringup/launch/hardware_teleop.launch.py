# Copyright (c) 2026 Peppermint Robotics. All rights reserved.
#
# Real hardware teleop — joystick driving only, no SLAM or Nav2.
# Launches: robot_state_publisher, cobra_driver, joy, teleop_twist_joy, twist_mux.
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    bringup_share = get_package_share_directory('pepperbyte_bringup')
    cobra_share = get_package_share_directory('cobra_driver')
    desc_share = get_package_share_directory('pepperbyte_description')

    cobra_params = os.path.join(cobra_share, 'config', 'cobra_driver_params.yaml')
    joy_config = os.path.join(bringup_share, 'config', 'joy_params.yaml')
    teleop_config = os.path.join(bringup_share, 'config', 'teleop_twist_joy_params.yaml')
    twist_mux_config = os.path.join(bringup_share, 'config', 'twist_mux.yaml')

    return LaunchDescription([
        # --- Robot description (URDF + robot_state_publisher) ---
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(desc_share, 'launch', 'description.launch.py')
            ),
            launch_arguments={'use_sim_time': 'false'}.items(),
        ),

        # --- Cobra driver (serial bridge to ESP32) ---
        # publish_tf=True here because there is no EKF in teleop-only mode
        Node(
            package='cobra_driver',
            executable='cobra_driver_node',
            name='cobra_driver_node',
            output='screen',
            parameters=[
                cobra_params,
                {'publish_tf': True, 'use_sim_time': False},
            ],
        ),

        # --- Joystick driver ---
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[joy_config],
        ),

        # --- Teleop twist joy (output → joy_vel for twist_mux) ---
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy_node',
            output='screen',
            parameters=[teleop_config],
            remappings=[('cmd_vel', 'joy_vel')],
        ),

        # --- Twist mux (joy_vel priority 20 → /cmd_vel) ---
        Node(
            package='twist_mux',
            executable='twist_mux',
            name='twist_mux',
            output='screen',
            parameters=[twist_mux_config],
            remappings=[('cmd_vel_out', '/cmd_vel')],
        ),
    ])
