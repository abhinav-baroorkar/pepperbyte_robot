# Copyright (c) 2026 Peppermint Robotics. All rights reserved.
#
# Launches the PepperByte web control app:
#   1. rosbridge_server (websocket on port 9090)
#   2. Flask web app (http://localhost:5000)
#
# Usage:
#   ros2 launch pepperbyte_app app.launch.py
import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess, LogInfo
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    app_share = get_package_share_directory('pepperbyte_app')
    webapp_dir = os.path.join(app_share, 'webapp')

    return LaunchDescription([
        LogInfo(msg='\n===  Peppermint Robotics Control App  ===\n'
                    '   http://localhost:5000\n'
                    '=========================================\n'),

        # rosbridge websocket server (port 9090)
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket',
            output='screen',
            parameters=[{'port': 9090}],
        ),

        # Flask web application
        ExecuteProcess(
            cmd=['python3', os.path.join(webapp_dir, 'app.py')],
            cwd=webapp_dir,
            output='screen',
        ),
    ])
