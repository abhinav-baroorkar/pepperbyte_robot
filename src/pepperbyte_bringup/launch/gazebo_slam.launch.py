# Copyright (c) 2026 Peppermint Robotics. All rights reserved.
#
# Gazebo Fortress simulation — SLAM mapping with EKF and RViz2.
# Drive with:  ros2 run teleop_twist_keyboard teleop_twist_keyboard
#
# Topic flow:
#   /scan      (Ignition GPU LiDAR) → [bridge] → slam_toolbox → /map
#                                              → rf2o         → /odom_rf2o
#   /odom      (Ignition DiffDrive) → [bridge] → ekf_node     → /odometry/filtered
#   /imu/data  (Ignition IMU)       → [bridge] → ekf_node
#   EKF fuses: /odom (vx) + /odom_rf2o (vx,vy,vyaw) + /imu/data (yaw rate, accel)
#   odom→base_link TF published by ekf_node
#   map→odom   TF published by slam_toolbox
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    desc_share   = get_package_share_directory('pepperbyte_description')
    slam_share   = get_package_share_directory('pepperbyte_slam')
    bringup_share = get_package_share_directory('pepperbyte_bringup')

    xacro_file  = os.path.join(desc_share,   'urdf',   'pepperbyte.urdf.xacro')
    world_file  = os.path.join(desc_share,   'worlds', 'commercial_building.sdf')
    slam_config = os.path.join(slam_share,   'config', 'slam_toolbox_online_async.yaml')
    ekf_config  = os.path.join(slam_share,   'config', 'robot_localization_ekf.yaml')
    rviz_config = os.path.join(bringup_share, 'rviz',  'slam.rviz')

    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use Gazebo Fortress simulation clock'
        ),

        # --- Gazebo Fortress: simple room world ---
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('ros_ign_gazebo'),
                    'launch',
                    'ign_gazebo.launch.py'
                )
            ),
            launch_arguments={'ign_args': f'-r {world_file}'}.items(),
        ),

        # --- robot_state_publisher ---
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

        # --- Spawn robot into Gazebo Fortress ---
        Node(
            package='ros_ign_gazebo',
            executable='create',
            name='spawn_pepperbyte',
            output='screen',
            arguments=[
                '-name', 'pepperbyte',
                '-topic', '/robot_description',
                '-x', '0',
                '-y', '0',
                '-z', '0.05',
            ],
        ),

        # --- ROS ↔ Ignition bridge ---
        Node(
            package='ros_ign_bridge',
            executable='parameter_bridge',
            name='ign_bridge',
            output='screen',
            arguments=[
                '/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
                '/odom@nav_msgs/msg/Odometry@ignition.msgs.Odometry',
                '/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
                '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
                '/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model',
                '/imu/data@sensor_msgs/msg/Imu[ignition.msgs.IMU',
                '/model/pepperbyte/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
            ],
        ),

        # --- Static TF alias: Ignition merges all fixed joints into the root
        #     link (base_footprint), so the LiDAR sensor's frame_id becomes
        #     "pepperbyte/base_footprint/lidar" in Ignition's scoped naming.
        #     This zero-offset static transform makes that frame findable in
        #     the ROS2 TF tree so slam_toolbox can process /scan. ---
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='lidar_frame_fix',
            output='screen',
            arguments=[
                '--frame-id', 'lidar_link',
                '--child-frame-id', 'pepperbyte/base_footprint/lidar',
                '--x', '0', '--y', '0', '--z', '0',
                '--roll', '0', '--pitch', '0', '--yaw', '0',
            ],
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
                'use_sim_time': use_sim_time,
            }],
        ),

        # --- slam_toolbox: online async mapping ---
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                slam_config,
                {'use_sim_time': use_sim_time},
            ],
        ),

        # --- robot_localization: EKF fusing /odom + /odom_rf2o + /imu/data ---
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[
                ekf_config,
                {'use_sim_time': use_sim_time, 'publish_tf': True},
            ],
        ),

        # --- Map cleaner: replaces unknown (-1) cells with free (0) on /map_clean ---
        Node(
            package='pepperbyte_bringup',
            executable='map_cleaner.py',
            name='map_cleaner',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),

        # --- RViz2 ---
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': use_sim_time}],
        ),
    ])
