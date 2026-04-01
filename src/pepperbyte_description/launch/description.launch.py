# Copyright (c) 2026 Peppermint Robotics. All rights reserved.
import os
import subprocess
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    xacro_file = os.path.join(
        get_package_share_directory('pepperbyte_description'),
        'urdf',
        'pepperbyte.urdf.xacro'
    )

    robot_description = subprocess.check_output(['xacro', xacro_file]).decode()

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true'
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': use_sim_time,
            }],
        ),
        # joint_state_publisher removed — cobra_driver publishes /joint_states
        # directly from wheel encoder data (odl/odr fields in ESP32 feedback).
    ])
