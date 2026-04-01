# Copyright (c) 2026 Peppermint Robotics. All rights reserved.
#
# Real hardware teleop — keyboard driving only, no SLAM or Nav2.
# Launches: robot_state_publisher, cobra_driver, teleop_twist_keyboard, twist_mux.
import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    cobra_share = get_package_share_directory('cobra_driver')
    desc_share = get_package_share_directory('pepperbyte_description')
    bringup_share = get_package_share_directory('pepperbyte_bringup')

    cobra_params = os.path.join(cobra_share, 'config', 'cobra_driver_params.yaml')
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

        # --- Keyboard teleop in a dedicated xterm window ---
        # Using ExecuteProcess instead of Node+prefix so we can fully control the
        # bash -c command string. Node prefix='xterm -e' breaks because launch
        # appends the executable path after the prefix — bash -c never sees it.
        ExecuteProcess(
            cmd=[
                'xterm', '-e', 'bash', '-c',
                'source /opt/ros/humble/setup.bash && '
                'source ~/pepperbyte_ws/pepperbyte_robot/install/setup.bash && '
                'ros2 run teleop_twist_keyboard teleop_twist_keyboard '
                '--ros-args -r cmd_vel:=joy_vel',
            ],
            output='screen',
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
