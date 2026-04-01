# Copyright (c) 2026 Peppermint Robotics. All rights reserved.
#
# ONE launch file to rule them all — full PepperByte simulation stack.
#
# Launches:
#   Gazebo Fortress → bridge → RSP → EKF → RF2O → slam_toolbox →
#   (15s delay) → Nav2 + twist_mux → joy + teleop →
#   rosbridge → Flask web app
#
# Drive manually: hold LB on F710 joystick, or use teleop_twist_keyboard
# Navigate autonomously: click on the map in the web app (http://localhost:5000)
# Or: ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=joy_vel

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess,
    IncludeLaunchDescription, LogInfo, TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    desc_share    = get_package_share_directory('pepperbyte_description')
    slam_share    = get_package_share_directory('pepperbyte_slam')
    nav_share     = get_package_share_directory('pepperbyte_navigation')
    bringup_share = get_package_share_directory('pepperbyte_bringup')
    app_share     = get_package_share_directory('pepperbyte_app')

    xacro_file   = os.path.join(desc_share,    'urdf',   'pepperbyte.urdf.xacro')
    world_file   = os.path.join(desc_share,    'worlds', 'commercial_building.sdf')
    slam_config  = os.path.join(slam_share,    'config', 'slam_toolbox_online_async.yaml')
    ekf_config   = os.path.join(slam_share,    'config', 'robot_localization_ekf.yaml')
    nav2_params  = os.path.join(nav_share,     'config', 'nav2_params.yaml')
    joy_params   = os.path.join(bringup_share, 'config', 'joy_params.yaml')
    teleop_params = os.path.join(bringup_share, 'config', 'teleop_twist_joy_params.yaml')
    webapp_dir   = os.path.join(app_share,     'webapp')

    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str,
    )

    # --- Nav2 + twist_mux (delayed 15s) -----------------------------------
    nav2_nodes = [
        # twist_mux: arbitrate between joystick and Nav2
        Node(
            package='twist_mux',
            executable='twist_mux',
            name='twist_mux',
            output='screen',
            parameters=[
                os.path.join(bringup_share, 'config', 'twist_mux.yaml'),
                {'use_sim_time': use_sim_time},
            ],
            remappings=[('cmd_vel_out', '/cmd_vel')],
        ),

        # Nav2: controller_server (outputs nav_vel → twist_mux)
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[nav2_params, {'use_sim_time': use_sim_time}],
            remappings=[('cmd_vel', 'nav_vel')],
        ),

        # Nav2: planner_server
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[nav2_params, {'use_sim_time': use_sim_time}],
        ),

        # Nav2: behavior_server (spin, backup, wait)
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[nav2_params, {'use_sim_time': use_sim_time}],
        ),

        # Nav2: bt_navigator
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[nav2_params, {'use_sim_time': use_sim_time}],
        ),

        # Nav2: lifecycle_manager
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[nav2_params, {'use_sim_time': use_sim_time}],
        ),
    ]

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use Gazebo Fortress simulation clock',
        ),

        LogInfo(msg='\n'
            '============================================\n'
            '   Peppermint Robotics Full System Launch\n'
            '   Web app:  http://localhost:5000\n'
            '============================================\n'),

        # --- Gazebo Fortress -----------------------------------------------
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('ros_ign_gazebo'),
                    'launch', 'ign_gazebo.launch.py',
                )
            ),
            launch_arguments={'ign_args': f'-r {world_file}'}.items(),
        ),

        # --- robot_state_publisher -----------------------------------------
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

        # --- Spawn robot ---------------------------------------------------
        Node(
            package='ros_ign_gazebo',
            executable='create',
            name='spawn_pepperbyte',
            output='screen',
            arguments=[
                '-name', 'pepperbyte',
                '-topic', '/robot_description',
                '-x', '0', '-y', '0', '-z', '0.05',
            ],
        ),

        # --- ROS <-> Ignition bridge ---------------------------------------
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

        # --- Static TF: Ignition scoped LiDAR frame -> lidar_link ---------
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

        # --- RF2O: LiDAR odometry ------------------------------------------
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

        # --- robot_localization: EKF ---------------------------------------
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

        # --- slam_toolbox --------------------------------------------------
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

        # --- Map cleaner ---------------------------------------------------
        Node(
            package='pepperbyte_bringup',
            executable='map_cleaner.py',
            name='map_cleaner',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),

        # --- Joy node (Logitech F710) --------------------------------------
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[
                joy_params,
                {'use_sim_time': use_sim_time},
            ],
        ),

        # --- teleop_twist_joy → publishes to /joy_vel ---------------------
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy_node',
            output='screen',
            parameters=[
                teleop_params,
                {'use_sim_time': use_sim_time},
            ],
            remappings=[('cmd_vel', 'joy_vel')],
        ),

        # --- rosbridge websocket (port 9090) -------------------------------
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket',
            output='screen',
            parameters=[{'port': 9090}],
        ),

        # --- Flask web app -------------------------------------------------
        ExecuteProcess(
            cmd=['python3', os.path.join(webapp_dir, 'app.py')],
            cwd=webapp_dir,
            output='screen',
        ),

        # --- Nav2 stack (delayed 15s) --------------------------------------
        TimerAction(
            period=15.0,
            actions=nav2_nodes,
        ),
    ])
