# Copyright (c) 2026 Peppermint Robotics. All rights reserved.
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('cobra_driver'),
        'config',
        'cobra_driver_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='cobra_driver',
            executable='cobra_driver_node',
            name='cobra_driver_node',
            output='screen',
            parameters=[config],
        ),
    ])
