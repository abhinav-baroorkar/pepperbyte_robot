# Copyright (c) 2026 Peppermint Robotics. All rights reserved.
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    slam_config = os.path.join(
        get_package_share_directory('pepperbyte_slam'),
        'config',
        'slam_toolbox_online_async.yaml'
    )

    ekf_config = os.path.join(
        get_package_share_directory('pepperbyte_slam'),
        'config',
        'robot_localization_ekf.yaml'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true'
        ),

        # slam_toolbox in online async mode
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[slam_config, {'use_sim_time': use_sim_time}],
        ),

        # robot_localization EKF — fuses wheel odom (+ IMU when enabled in config)
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config, {'use_sim_time': use_sim_time}],
            remappings=[('odometry/filtered', '/odometry/filtered')],
        ),
    ])
