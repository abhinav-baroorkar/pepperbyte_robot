# Copyright (c) 2026 Peppermint Robotics. All rights reserved.
#
# Real hardware SLAM mapping — no Gazebo, no simulation.
# Launches: robot_state_publisher, cobra_driver, rplidar, rf2o, EKF,
#           slam_toolbox, joy, teleop_twist_joy, twist_mux.
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
    slam_share = get_package_share_directory('pepperbyte_slam')

    cobra_params = os.path.join(cobra_share, 'config', 'cobra_driver_params.yaml')
    slam_config = os.path.join(slam_share, 'config', 'slam_toolbox_online_async.yaml')
    ekf_config = os.path.join(slam_share, 'config', 'robot_localization_ekf.yaml')
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
        # publish_tf=False because EKF publishes odom→base_link
        Node(
            package='cobra_driver',
            executable='cobra_driver_node',
            name='cobra_driver_node',
            output='screen',
            parameters=[
                cobra_params,
                {'use_sim_time': False},
            ],
        ),

        # --- RPLidar driver ---
        Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='rplidar_node',
            output='screen',
            parameters=[{
                'serial_port': '/dev/rplidar',
                'serial_baudrate': 256000,
                'frame_id': 'lidar_link',
                'angle_compensate': True,
                'scan_mode': 'DenseBoost',
            }],
        ),

        # --- RF2O: scan-to-scan LiDAR odometry → /odom_rf2o ---
        Node(
            package='rf2o_laser_odometry',
            executable='rf2o_laser_odometry_node',
            name='rf2o_laser_odometry',
            output='screen',
            parameters=[{
                'laser_scan_topic': '/scan',
                'odom_topic': '/odom_rf2o',
                'publish_tf': False,
                'base_frame_id': 'base_link',
                'odom_frame_id': 'odom',
                'init_pose_from_topic': '',
                'freq': 10.0,
                'use_sim_time': False,
            }],
        ),

        # --- robot_localization EKF — fuses /odom + /odom_rf2o ---
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[
                ekf_config,
                {'use_sim_time': False, 'publish_tf': True},
            ],
            remappings=[('odometry/filtered', '/odometry/filtered')],
        ),

        # --- slam_toolbox: online async mapping ---
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                slam_config,
                {'use_sim_time': False},
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
