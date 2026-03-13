# Copyright (c) 2026 Peppermint Robotics. All rights reserved.
#
# Bare teleop for testing: cobra_driver + joy + teleop only, no SLAM or Nav2.
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
        # No EKF in teleop_only mode, so cobra_driver must publish odom->base_link TF
        Node(
            package='cobra_driver',
            executable='cobra_driver_node',
            name='cobra_driver_node',
            output='screen',
            parameters=[
                os.path.join(
                    get_package_share_directory('cobra_driver'),
                    'config', 'cobra_driver_params.yaml'
                ),
                {'publish_tf': True},
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

        # --- Teleop twist joy (direct to /cmd_vel — no twist_mux needed) ---
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy_node',
            output='screen',
            parameters=[teleop_config],
        ),
    ])
