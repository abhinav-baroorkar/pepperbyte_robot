# PepperByte Robot Full Development & Testing Plan
Peppermint Robotics | 2026
Platform: Waveshare Cobra Flex 4WD + NVIDIA Jetson Orin Nano 8GB + ROS2 Humble

---

## Project Overview
PepperByte is a compact 2D mapping scout robot built on the Waveshare Cobra Flex 4WD chassis, powered by an NVIDIA Jetson Orin Nano 8GB running ROS2 Humble. Its primary mission is to rapidly map indoor environments using a 2D occupancy grid before deploying full-size scrubber dryer robots. This document covers the complete development roadmap from simulation to hardware deployment, including all software phases, testing procedures, connection wiring, and the mapping control application.

---

## Hardware Summary

| Component | Specification | ROS2 Interface |
|-----------|---------------|----------------|
| Chassis | Waveshare Cobra Flex 4WD, 235×173×101mm | Serial via cobra_driver |
| MCU | ESP32-S3 motor controller | /dev/ttyUSB0 or /dev/cobra_flex |
| Compute | NVIDIA Jetson Orin Nano 8GB | — |
| LiDAR | RPLidar 2D, 12m range, 360°/scan | /scan |
| IMU | BNO055 9-DOF (future) | /imu/data (disabled) |
| Camera | OAK-D Series 2 depth camera | Not yet integrated |
| Gamepad | Logitech F710 XInput | /dev/input/js0 via joy_node |
| Battery | 2× Zeee 3S 5200mAh LiPo, EC5 connector | /battery_voltage |
| Power (motors) | 11.1V nominal, 12.6V max from LiPo | Barrel jack to chassis |
| Power (Jetson) | 5V via DC-DC buck converter (future) | USB-C to Jetson |

---

## Repository Structure

| Package | Language | Purpose |
|---------|----------|---------|
| cobra_driver | C++ | Serial bridge to ESP32 — only package that talks to hardware |
| pepperbyte_description | XML/Xacro | URDF robot model — geometry, links, joints, sensors |
| pepperbyte_slam | Python/YAML | SLAM (slam_toolbox) + EKF state estimation |
| pepperbyte_navigation | Python/YAML | Nav2 autonomous navigation stack |
| pepperbyte_bringup | Python/YAML | Master launch files + joystick config + twist_mux |

---

## Phase 1 — Gazebo Simulation
All software development and testing begins in simulation before any hardware is touched. This phase establishes a fully functional Gazebo Fortress simulation of PepperByte that mirrors the real robot as closely as possible.

### 1.1 Gazebo Fortress Installation
Gazebo Fortress is the officially supported simulator for ROS2 Humble. Gazebo Classic reached end-of-life in January 2025 and must not be used for new development.

```bash
sudo apt-get install -y ignition-fortress
sudo apt-get install -y ros-humble-ros-ign-bridges
sudo apt-get install -y ros-humble-ros-ign-gazebo
sudo apt-get install -y ros-humble-ros-ign-interfaces
sudo apt-get install -y ros-humble-teleop-twist-keyboard
sudo apt-get install -y ros-humble-nav2-map-server
```

> **NOTE:** Gazebo Harmonic can be used with Humble but requires non-official packages from packages.osrfoundation.org and conflicts with ros-humble-ros-gz* packages. Stick with Fortress.

### 1.2 URDF Updates for Gazebo Fortress

#### Differential Drive Plugin
- Replaces cobra_driver in simulation. Accepts `/cmd_vel` and publishes `/odom` and `odom→base_link` TF.
- Plugin: `ignition-gazebo-diff-drive-system`
- Left joints: `front_left_wheel_joint`, `rear_left_wheel_joint`
- Right joints: `front_right_wheel_joint`, `rear_right_wheel_joint`
- Wheel separation: 0.2285m (track width)
- Wheel radius: 0.03725m
- Command topic: `/cmd_vel`
- Odometry topic: `/odom`
- `publish_odom_tf: true`

#### GPU LiDAR Plugin
- Simulates the RPLidar sensor, publishing to `/scan` as `sensor_msgs/LaserScan`.
- Type: `gpu_lidar` on `lidar_link`
- 360° scan, 1° resolution
- Range: 0.15m min, 12.0m max
- Update rate: 10 Hz
- Gaussian noise: mean 0.0, stddev 0.01
- Topic: `/scan`

#### Joint State Publisher Plugin
- Publishes wheel joint states so RViz and robot_state_publisher can render the robot correctly.
- Plugin: `ignition-gazebo-joint-state-publisher-system`

#### Gazebo Colours

| Link | Colour | RGBA |
|------|--------|------|
| base_link | Dark Grey | 0.3 0.3 0.3 1 |
| All 4 wheel links | Black | 0.1 0.1 0.1 1 |
| lidar_link | Green | 0.2 0.8 0.2 1 |
| imu_link | Blue | 0.2 0.2 0.8 1 |
| camera_link | Black | 0.1 0.1 0.1 1 |

### 1.3 Simple Room World
File: `pepperbyte_description/worlds/simple_room.sdf`
- Room size: 10m × 10m
- Walls: 4 walls, 0.2m thick, 2.0m tall
- Ground plane with friction
- Ambient lighting + directional sun light
- 2 red cylindrical obstacles (radius 0.15m) for SLAM testing

### 1.4 Launch Files

| Launch File | Purpose | Nodes Started |
|-------------|---------|---------------|
| `gazebo_teleop.launch.py` | Manual driving in Gazebo | Gazebo Fortress, RSP, spawn, ros_ign_bridge |
| `gazebo_slam.launch.py` | SLAM mapping in Gazebo | + slam_toolbox, EKF, RViz2 |
| `gazebo_nav.launch.py` | Autonomous Nav2 in Gazebo | + full Nav2 stack |
| `gazebo_full.launch.py` | Everything together | All nodes, joystick override enabled |

### 1.5 Topic Bridging (Gazebo ↔ ROS2)

| ROS2 Topic | Direction | Type |
|-----------|-----------|------|
| /cmd_vel | ROS2 → Gazebo | geometry_msgs/Twist |
| /odom | Gazebo → ROS2 | nav_msgs/Odometry |
| /scan | Gazebo → ROS2 | sensor_msgs/LaserScan |
| /clock | Gazebo → ROS2 | rosgraph_msgs/Clock |
| /joint_states | Gazebo → ROS2 | sensor_msgs/JointState |

---

## Phase 2 — SLAM Mapping in Simulation

### 2.1 SLAM Stack Overview

| Node | Package | Role |
|------|---------|------|
| `async_slam_toolbox_node` | slam_toolbox | Graph-based SLAM using scan matching (Ceres solver) |
| `ekf_node` | robot_localization | Fuses /odom into smooth /odometry/filtered |
| `robot_state_publisher` | robot_state_publisher | Broadcasts static TF from URDF |
| `ros_ign_bridge` | ros_ign_bridge | Bridges /scan and /odom from Gazebo to ROS2 |

### 2.2 TF Chain Verification
The following TF chain must be complete and error-free before SLAM will work:

```
map → odom → base_link → {front_left_wheel_link, front_right_wheel_link,
                           rear_left_wheel_link, rear_right_wheel_link,
                           lidar_link, imu_link, camera_link}
```

Verification commands:
```bash
ros2 run tf2_tools view_frames    # generates a PDF of the full TF tree
ros2 topic echo /tf               # confirms transforms are publishing
```

### 2.3 Key SLAM Parameters

| Parameter | Value | Notes |
|-----------|-------|-------|
| map_resolution | 0.05m | 5cm/cell — good balance of detail vs memory |
| max_laser_range | 12.0m | Matches RPLidar spec |
| map_update_interval | 5.0s | How often the map is updated |
| minimum_travel_distance | 0.1m | New node after 10cm travel |
| minimum_travel_heading | 0.175 rad | New node after 10° rotation |
| loop_search_maximum_distance | 3.0m | Loop closure search radius |
| loop_match_minimum_chain_size | 10 | Min scan chain for loop closure |
| mode | mapping | Use localization mode after map is saved |

### 2.4 Map Saving
```bash
ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map
```
Produces:
- `~/maps/my_map.pgm` — greyscale occupancy grid image
- `~/maps/my_map.yaml` — metadata (resolution, origin, thresholds)

---

## Phase 3 — Autonomous Navigation in Simulation

### 3.1 Nav2 Stack Components

| Node | Role |
|------|------|
| planner_server | Global path planning (NavFn/Dijkstra) |
| controller_server | Path tracking (Regulated Pure Pursuit) |
| recoveries_server | Recovery behaviours: Spin, BackUp, Wait |
| bt_navigator | Orchestrates via Behaviour Tree |
| lifecycle_manager | Manages lifecycle of all Nav2 nodes |
| map_server | Loads the saved .yaml/.pgm map |
| amcl | Adaptive Monte Carlo Localisation |

### 3.2 Key Nav2 Parameters

| Parameter | Value | Notes |
|-----------|-------|-------|
| desired_linear_vel | 0.3 m/s | Conservative indoor speed |
| max_linear_vel | 0.5 m/s | Turbo mode ceiling |
| xy_goal_tolerance | 0.15m | Position tolerance at goal |
| yaw_goal_tolerance | 0.25 rad | ~14° heading tolerance |
| robot_radius | 0.13m | Costmap inflation input |
| inflation_radius | 0.25m | Obstacle clearance buffer |
| local_costmap_size | 3×3m | Rolling window, 5cm resolution |
| allow_reversing | false | Forward-only motion |

### 3.3 Sending Navigation Goals
- **RViz2** — Nav2 Goal tool (click and drag on the map)
- **CLI** — `ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose`
- **App** — via the Mapping Control App UI (Phase 4)

### 3.4 Obstacles World
File: `pepperbyte_description/worlds/obstacles_room.sdf`
- Same 10×10m room
- 6–8 box obstacles of varying sizes
- Narrow corridor section for inflation radius testing

---

## Phase 4 — Mapping Control Application
A locally-hosted web application running on Ubuntu. No cloud dependency.

### 4.1 Technology Stack

| Component | Technology |
|-----------|-----------|
| Backend | Python + Flask |
| Frontend | React + Tailwind CSS |
| ROS2 bridge | rclpy |
| Map display | Leaflet.js or Canvas |
| Real-time updates | WebSocket (Flask-SocketIO) |
| Hosting | localhost:5000 |

### 4.2 Application Features
- **Robot Control Panel**: Start/Stop Mapping, Save Map, Load Map, Emergency Stop
- **Mode Switcher**: Teleop / Autonomous / Mapping
- **Live Map View**: /map rendered in real-time, robot overlay, click-to-navigate
- **Status Dashboard**: battery voltage, robot speed, node health, LiDAR mini-view
- **Map Management**: list, preview, delete, rename, export maps from `~/maps/`

### 4.3 File Structure

| File | Purpose |
|------|---------|
| `app.py` | Flask server + rclpy integration |
| `ros_bridge.py` | ROS2 subscriber/publisher wrappers |
| `static/index.html` | Main app page |
| `static/js/app.jsx` | React frontend |
| `static/js/MapView.jsx` | Occupancy grid renderer |
| `static/js/ControlPanel.jsx` | Robot control buttons |
| `static/js/StatusBar.jsx` | Live status dashboard |

---

## Phase 5 — Hardware Integration

### 5.1 Git Workflow
```bash
# On laptop:
git push origin main
# On Jetson:
ssh user@<jetson_ip>
cd ~/pepperbyte_ws && git pull
colcon build --symlink-install
source install/setup.bash
```

### 5.2 Hardware Testing Order

| Step | Component | Test | Success Criteria |
|------|-----------|------|------------------|
| 2A | udev rule | Install 99-cobra-flex.rules | /dev/cobra_flex symlink exists |
| 2B | ESP32 comms | Connect USB | Device appears, no permission errors |
| 2C | Motor power | Connect 12V to chassis | ESP32 LEDs on, no smoke |
| 2D | Encoder data | Launch teleop_only, echo /odom | /odom publishes at correct rate |
| 2E | Motor drive | Teleop keyboard | Robot moves, odom updates |
| 2F | Battery voltage | echo /battery_voltage | Reading ~11.1–12.6V |
| 3A | LiDAR USB | Connect RPLidar | /dev/ttyUSB1 appears |
| 3B | LiDAR scan | Launch RPLidar driver | /scan publishes 360 readings |
| 3C | LiDAR RViz | Launch slam | Laser scan visible around robot |
| 4A | SLAM | Launch mapping_teleop, drive around | Map builds in RViz |
| 4B | Save map | map_saver or app | .pgm and .yaml saved |
| 4C | Nav2 | Launch mapping_nav, send goal | Robot navigates autonomously |
| 4D | Full stack | Launch everything | App controls robot end-to-end |

### 5.3 Power Wiring Plan

```
Battery EC5 (+) ──┬──> Chassis barrel jack (motor power, 11.1–12.6V)
                  └──> DC-DC buck converter ──> Jetson USB-C (5V 4A)
Battery EC5 (-) ──> common ground
```

> **WARNING:** Always connect USB to Jetson BEFORE connecting the battery.
> **WARNING:** Use a 10A fuse on the positive line between battery and chassis.

### 5.4 Connection Order (Every Session)
1. Check battery voltage with LiPo checker (must be >10.5V / >3.5V per cell)
2. Connect USB from ESP32 to Jetson
3. SSH into Jetson or connect monitor, launch ROS2
4. Verify `/dev/cobra_flex` exists
5. Connect battery barrel jack to chassis
6. Verify `/battery_voltage` is publishing
7. Proceed with testing

**Disconnect order (reverse):**
1. `ros2 topic pub /cmd_vel geometry_msgs/Twist '{}'` (emergency stop)
2. Disconnect battery
3. Disconnect USB

---

## Phase 6 — Future Enhancements

### 6.1 IMU Integration
The BNO055 is physically installed but its EKF input is commented out.
- Enable `/imu/data` input in `pepperbyte_slam/config/robot_localization_ekf.yaml`
- Install: `ros-humble-bno055`
- Calibrate on flat surface before use

### 6.2 OAK-D Camera Integration
- Install DepthAI ROS2 driver
- Add depth obstacle avoidance layer to Nav2 local costmap
- Future: visual odometry, person-following, object detection

### 6.3 Multi-Floor Mapping
- Map stitching between floors
- Floor-level metadata in saved maps
- Floor selector in Mapping Control App

### 6.4 Fleet Management
- Centralised map server for multiple robots
- Conflict resolution when robot paths overlap
- Web dashboard showing all robot locations

---

## Appendix — Key ROS2 Commands

### A. Topic Monitoring
```bash
ros2 topic list
ros2 topic echo /odom
ros2 topic echo /battery_voltage
ros2 topic echo /scan
ros2 topic hz /scan
ros2 topic hz /odom
ros2 run tf2_tools view_frames
```

### B. Launch Commands
```bash
ros2 launch pepperbyte_bringup gazebo_teleop.launch.py   # Gazebo + keyboard teleoperation
ros2 launch pepperbyte_bringup gazebo_slam.launch.py     # Gazebo + SLAM mapping
ros2 launch pepperbyte_bringup gazebo_nav.launch.py      # Gazebo + Nav2 autonomous
ros2 launch pepperbyte_bringup teleop_only.launch.py     # Real robot, keyboard only
ros2 launch pepperbyte_bringup mapping_teleop.launch.py  # Real robot, SLAM + teleoperation
ros2 launch pepperbyte_bringup mapping_nav.launch.py     # Real robot, SLAM + Nav2
```

### C. Map Saving & Loading
```bash
ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map
ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=~/maps/my_map.yaml
```

### D. Emergency Commands
```bash
ros2 topic pub /cmd_vel geometry_msgs/Twist '{}'         # Send zero velocity
ros2 topic pub /cmd_vel geometry_msgs/Twist '{}' --once  # One-shot stop
# Ctrl+C in launch terminal — kill all nodes launched by that file
```

### E. Joystick Configuration (Logitech F710)

| Control | Axis/Button | Action |
|---------|-------------|--------|
| Left stick vertical | Axis 1 | Linear X (forward/back) |
| Left stick horizontal | Axis 0 | Angular yaw (turn) |
| LB button | Button 4 | Enable button — MUST hold to drive |
| RB button | Button 5 | Turbo mode |
| Normal speed | — | 0.3 m/s linear, 1.0 rad/s angular |
| Turbo speed | — | 0.5 m/s linear, 1.5 rad/s angular |
