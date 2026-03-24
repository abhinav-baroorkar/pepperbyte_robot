# Copyright (c) 2026 Peppermint Robotics. All rights reserved.
#
# Gazebo Fortress simulation — teleop only (no cobra_driver, no twist_mux).
# Drive with:  ros2 run teleop_twist_keyboard teleop_twist_keyboard
#
# Topic flow:
#   teleop_keyboard → /cmd_vel → [ros_ign_bridge] → /cmd_vel (Ignition)
#                                                           ↓
#                                               DiffDrive plugin (Fortress)
#                                                           ↓
#   /odom (ROS2) ← [ros_ign_bridge] ← /odom (Ignition)
#   /scan (ROS2) ← [ros_ign_bridge] ← /scan (Ignition GPU LiDAR)
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

    desc_share = get_package_share_directory('pepperbyte_description')

    xacro_file = os.path.join(desc_share, 'urdf', 'pepperbyte.urdf.xacro')
    world_file = os.path.join(desc_share, 'worlds', 'commercial_building.sdf')

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
        # /cmd_vel      : ROS2 Twist      → Ignition   (teleop drives the sim)
        # /odom         : Ignition        → ROS2       (diff drive odometry)
        # /scan         : Ignition        → ROS2       (GPU LiDAR → LaserScan)
        # /clock        : Ignition        → ROS2       (sim time)
        # /joint_states : Ignition        → ROS2       (wheel angles for RSP)
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
            ],
        ),
    ])
