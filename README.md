# PepperByte — Scout Mapping Robot

PepperByte is a compact autonomous 2D mapping scout robot built by **Peppermint Robotics**. Its purpose is to quickly map indoor environments (generating 2D occupancy grid maps) before deploying full-size autonomous scrubber dryers.

## Hardware Overview

PepperByte is built on a Waveshare Cobra Flex 4WD differential-drive chassis with an ESP32-S3 motor controller, powered by an NVIDIA Jetson Orin Nano 8GB running Ubuntu 22.04 and ROS2 Humble. It carries an RPLidar 2D scanner (with a future upgrade path to a Robosense 36-beam 3D LiDAR), a BNO055 9-DOF IMU, and an OAK-D Series 2 depth camera for future perception tasks. Two Zeee 3S 5200mAh LiPo batteries in parallel supply power through a 5V 10A buck converter.

## Operating Modes

| Mode | Launch file | Description |
|------|------------|-------------|
| **Teleop Mapping** | `mapping_teleop.launch.py` | Drive with a Logitech F710 gamepad while slam_toolbox builds the map |
| **Autonomous Goal Mapping** | `mapping_nav.launch.py` | Send Nav2 goal poses in RViz; joystick override available |
| **Teleop Only** | `teleop_only.launch.py` | Bare joystick driving for hardware checkout (no SLAM) |

## Build

```bash
cd ~/pepperbyte_robot
colcon build --symlink-install
source install/setup.bash
```

## Launch

```bash
# Mode 1: Teleop mapping
ros2 launch pepperbyte_bringup mapping_teleop.launch.py

# Mode 2: Autonomous goal-pose mapping
ros2 launch pepperbyte_bringup mapping_nav.launch.py

# Mode 3: Teleop only (testing)
ros2 launch pepperbyte_bringup teleop_only.launch.py
```

## Viewing the Map on a Laptop

On the laptop (same network, same `ROS_DOMAIN_ID`):

```bash
export ROS_DOMAIN_ID=42
rviz2 -d ~/pepperbyte_robot/src/pepperbyte_bringup/rviz/mapping.rviz
```

## Saving the Map

Once you are satisfied with the map:

```bash
mkdir -p ~/maps
ros2 run nav2_map_server map_saver_cli -f ~/maps/site_name
```

This saves `site_name.pgm` and `site_name.yaml` which can be loaded later for pure navigation.

## Packages

| Package | Description |
|---------|-------------|
| `cobra_driver` | Serial bridge to the Cobra Flex ESP32 — odometry, cmd_vel, battery voltage |
| `pepperbyte_description` | URDF/xacro robot model and robot_state_publisher launch |
| `pepperbyte_slam` | slam_toolbox + robot_localization EKF configuration |
| `pepperbyte_navigation` | Nav2 stack configuration (planner, controller, costmaps) |
| `pepperbyte_bringup` | Master launch files, twist_mux, joystick configs, RViz preset |

## Deploying to the Jetson

```bash
./scripts/deploy.sh jetson 192.168.1.100
```
