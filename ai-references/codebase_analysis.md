# PepperByte Robot — Codebase Analysis

> **Robot:** PepperByte — compact 2D mapping scout by Peppermint Robotics
> **Platform:** Waveshare Cobra Flex 4WD + NVIDIA Jetson Orin Nano 8GB + ROS2 Humble
> **Purpose:** Rapidly map indoor environments (2D occupancy grid) before deploying full-size scrubber dryers

---

## Repository Structure

```
pepperbyte_robot/
├── scripts/                        # Utility shell scripts
└── src/
    ├── cobra_driver/               # C++ serial bridge to ESP32 motor controller
    ├── pepperbyte_description/     # URDF robot model
    ├── pepperbyte_slam/            # SLAM + EKF configuration
    ├── pepperbyte_navigation/      # Nav2 stack configuration
    └── pepperbyte_bringup/         # Master launch files + joystick configs
```

---

## Package-by-Package Analysis

---

### 1. `cobra_driver`
**Language:** C++
**Role:** Serial hardware bridge — the only package that talks to the physical robot.

#### What it does
- Opens a POSIX serial connection to the **Waveshare Cobra Flex ESP32-S3** motor controller.
- On startup, sends `{"T":131,"cmd":1}` to enable continuous JSON feedback from the ESP32.
- Runs a **background thread** to continuously read newline-delimited JSON from the ESP32.
- A **keepalive timer** fires every `cmd_vel_timeout_ms/2` ms (default 1 s). If no `cmd_vel` has been received in the full timeout window (2 s), it sends a zero-velocity command to prevent the ESP32 watchdog from cutting power.
- On shutdown, sends `{"T":0}` (emergency stop) before closing the port.

#### Inputs
| Source | Type | Topic/Interface | Description |
|--------|------|-----------------|-------------|
| ROS2 subscriber | `geometry_msgs/Twist` | `/cmd_vel` | Velocity commands from twist_mux or teleop |

#### Outputs
| Type | Topic | Description |
|------|-------|-------------|
| `nav_msgs/Odometry` | `/odom` | Differential-drive wheel odometry (x, y, yaw, vx, vyaw) |
| `std_msgs/Float32` | `/battery_voltage` | Battery voltage decoded from ESP32 feedback (`v/100`) |
| TF transform | `odom → base_link` | Only when `publish_tf: true` (disabled when EKF is running) |

#### Serial Protocol (ESP32 JSON)
| Direction | Message | Meaning |
|-----------|---------|---------|
| → ESP32 | `{"T":131,"cmd":1}` | Enable continuous feedback stream |
| → ESP32 | `{"T":13,"X":<vx>,"Z":<wz>}` | Set linear (m/s) and angular (rad/s) velocity |
| → ESP32 | `{"T":0}` | Emergency stop |
| ← ESP32 | `{"T":1001,"M1":…,"odl":<cm>,"odr":<cm>,"v":<v×100>}` | Base-info feedback: encoder ticks and battery |

#### Odometry Kinematics (differential drive, midpoint integration)
```
delta_left_m  = (odl_cm - prev_odl_cm) × 0.01
delta_right_m = (odr_cm - prev_odr_cm) × 0.01
delta_s       = (delta_right + delta_left) / 2
delta_theta   = (delta_right - delta_left) / track_width   [track_width = 0.159 m]
x      += delta_s × cos(theta + delta_theta/2)
y      += delta_s × sin(theta + delta_theta/2)
theta  += delta_theta
```

#### Key Parameters (`cobra_driver_params.yaml`)
| Parameter | Default | Notes |
|-----------|---------|-------|
| `serial_port` | `/dev/ttyUSB0` | Use `/dev/cobra_flex` after udev rule install |
| `baud_rate` | `115200` | 8N1 |
| `wheel_diameter` | `0.0739 m` | 73.9 mm wheels |
| `track_width` | `0.159 m` | Distance between left/right wheels |
| `publish_tf` | `false` | Disabled in SLAM/Nav modes; EKF publishes odom TF instead |
| `cmd_vel_timeout_ms` | `2000` | Keepalive fires every 1000 ms if idle |

#### Files
- `src/cobra_driver_node.cpp` — ROS2 node (subscriber, publishers, TF broadcaster, serial thread, odometry math)
- `src/serial_port.cpp` / `include/cobra_driver/serial_port.hpp` — minimal POSIX serial wrapper (no external libs; uses `termios`, `select()`)
- `config/99-cobra-flex.rules` — udev rule: creates `/dev/cobra_flex` symlink for the ESP32-S3 (Espressif VID `303a`, PID `1001`)

---

### 2. `pepperbyte_description`
**Language:** XML/Xacro
**Role:** Robot model — defines geometry, links, joints, and sensor placements.

#### What it does
- Provides the URDF via `pepperbyte.urdf.xacro`.
- Launches `robot_state_publisher` (broadcasts static TF tree from URDF) and `joint_state_publisher` (publishes wheel joint states at zero since the ESP32 doesn't report individual joint angles).

#### Robot Model Structure (`pepperbyte.urdf.xacro`)
```
base_footprint  (ground-level virtual link)
  └─ base_link        [chassis box: 250×200×80 mm, 2.0 kg]
        ├─ left_wheel_link   [cylinder r=36.95mm, w=30mm, continuous joint, axis Y]
        ├─ right_wheel_link  [cylinder r=36.95mm, w=30mm, continuous joint, axis Y]
        ├─ lidar_link        [cylinder r=35mm h=40mm, fixed, 150mm above base_link]
        ├─ imu_link          [box 20×20×5mm, fixed, at top of chassis]
        └─ camera_link       [box 30×100×25mm, fixed, 100mm up + 45° pitch-down, front face]
```

#### Outputs (via `description.launch.py`)
| Node | Output Topic/TF | Description |
|------|-----------------|-------------|
| `robot_state_publisher` | `/robot_description`, TF static frames | Publishes URDF + static transforms |
| `joint_state_publisher` | `/joint_states` | Publishes wheel joint states (zeros) |

---

### 3. `pepperbyte_slam`
**Language:** Python (launch), YAML (config)
**Role:** SLAM and state estimation — builds the map and provides fused odometry.

#### What it does
Launches two nodes:

**A. `slam_toolbox` (async_slam_toolbox_node)**
- Subscribes to `/scan` (LiDAR) and the TF tree (to get robot pose in odom frame).
- Performs online, asynchronous graph-based SLAM using scan matching (Ceres solver).
- Publishes `/map` (`nav_msgs/OccupancyGrid`) and the `map → odom` TF.
- Loop closure is enabled: searches up to 3 m, requires chain of ≥10 scans.
- Map resolution: 5 cm/cell; max laser range: 12 m; updates every 5 s.
- Triggers a new node when the robot moves ≥0.1 m or rotates ≥10°.

**B. `robot_localization` EKF (ekf_node)**
- Fuses sensor inputs into a smooth odometry estimate.
- **Current inputs:** `/odom` (wheel odometry) — position x,y; orientation yaw; velocity vx,vy,vyaw.
- **IMU input** (`/imu/data` from BNO055) is **commented out** — placeholder for future enable.
- Publishes `/odometry/filtered` and the `odom → base_link` TF (so `cobra_driver`'s `publish_tf` is false).
- Runs at 30 Hz, 2D mode only (z/roll/pitch ignored).

#### Inputs
| Topic | Type | Producer |
|-------|------|---------|
| `/scan` | `sensor_msgs/LaserScan` | RPLidar ROS driver (external, not in this repo) |
| `/odom` | `nav_msgs/Odometry` | `cobra_driver` |
| `/imu/data` *(disabled)* | `sensor_msgs/Imu` | BNO055 driver (future) |

#### Outputs
| Topic / TF | Description |
|------------|-------------|
| `/map` | 2D occupancy grid (5 cm resolution) |
| TF: `map → odom` | Published by slam_toolbox |
| `/odometry/filtered` | EKF-fused odometry |
| TF: `odom → base_link` | Published by EKF node |

---

### 4. `pepperbyte_navigation`
**Language:** Python (launch), YAML (config)
**Role:** Autonomous point-to-point navigation using the Nav2 stack.

#### What it does
Launches the full Nav2 pipeline:

| Node | Role |
|------|------|
| `controller_server` | Executes path tracking using **Regulated Pure Pursuit** controller |
| `planner_server` | Global path planning using **NavFn** (Dijkstra, not A*) |
| `recoveries_server` | Recovery behaviours: Spin, BackUp, Wait |
| `bt_navigator` | Orchestrates everything via a Behaviour Tree |
| `lifecycle_manager` | Manages lifecycle of all Nav2 nodes (autostart) |

#### Inputs
| Topic | Type | Source |
|-------|------|--------|
| `/scan` | `sensor_msgs/LaserScan` | RPLidar |
| `/odometry/filtered` | `nav_msgs/Odometry` | EKF (pepperbyte_slam) |
| `/map` | `nav_msgs/OccupancyGrid` | slam_toolbox |
| Goal pose | via `bt_navigator` action server | RViz "Nav2 Goal" tool |

#### Outputs
| Topic | Type | Description |
|-------|------|-------------|
| `nav_vel` | `geometry_msgs/Twist` | Nav2 velocity commands (fed into twist_mux) |

#### Key Nav2 Tuning (for this robot)
| Parameter | Value | Notes |
|-----------|-------|-------|
| `desired_linear_vel` | 0.3 m/s | Conservative indoor speed |
| `xy_goal_tolerance` | 0.15 m | Position tolerance |
| `yaw_goal_tolerance` | 0.25 rad (~14°) | Heading tolerance |
| `robot_radius` | 0.13 m | Used for costmap inflation |
| `inflation_radius` | 0.25 m | Obstacle clearance buffer |
| `local_costmap size` | 3×3 m rolling window | 5 cm resolution |
| `global_costmap` | Full map frame | Tracks unknown space |
| `allow_reversing` | false | Forward-only motion |

> **Important:** `controller_server` remaps its output `cmd_vel → nav_vel` so it does not bypass twist_mux.

---

### 5. `pepperbyte_bringup`
**Language:** Python (launch), YAML (config)
**Role:** Master launch files and input multiplexing. The entry point for all operating modes.

#### Operating Modes

| Mode | Launch File | What's Started |
|------|-------------|----------------|
| **Teleop Mapping** | `mapping_teleop.launch.py` | description + cobra_driver + SLAM + joy + teleop_twist_joy + twist_mux |
| **Autonomous Mapping** | `mapping_nav.launch.py` | description + cobra_driver + SLAM + Nav2 + twist_mux + (optional joy override) |
| **Teleop Only** | `teleop_only.launch.py` | description + cobra_driver (`publish_tf=true`) + joy + teleop_twist_joy |

#### Velocity Multiplexing (`twist_mux`)
```
joy_vel   (priority 20, timeout 0.5s) ─┐
                                        ├→ twist_mux → /cmd_vel → cobra_driver
nav_vel   (priority 10, timeout 0.5s) ─┘
```
- Joystick **always overrides** Nav2 (higher priority number wins).
- In `teleop_only` mode there is no twist_mux — `teleop_twist_joy` writes directly to `/cmd_vel`.

#### Joystick Config (Logitech F710, XInput mode)
| Control | Mapping |
|---------|---------|
| Left stick vertical (axis 1) | Linear X (forward/back) |
| Left stick horizontal (axis 0) | Angular yaw (turn) |
| LB button (4) | Enable button — must hold to drive |
| RB button (5) | Turbo mode |
| Normal max speed | 0.3 m/s linear, 1.0 rad/s angular |
| Turbo max speed | 0.5 m/s linear, 1.5 rad/s angular |

---

## Full Topic & TF Map

### Topics

```
/scan              ← RPLidar driver (external)
/joy               ← joy_node → teleop_twist_joy_node
joy_vel            ← teleop_twist_joy_node → twist_mux
nav_vel            ← Nav2 controller_server → twist_mux
/cmd_vel           ← twist_mux → cobra_driver_node
/odom              ← cobra_driver_node → EKF
/battery_voltage   ← cobra_driver_node
/odometry/filtered ← EKF → Nav2
/map               ← slam_toolbox → Nav2 global costmap
/robot_description ← robot_state_publisher
/joint_states      ← joint_state_publisher
```

### TF Tree

```
map
 └─ odom              (published by slam_toolbox)
      └─ base_link    (published by EKF in SLAM/Nav modes; by cobra_driver in teleop_only)
           ├─ left_wheel_link
           ├─ right_wheel_link
           ├─ lidar_link
           ├─ imu_link
           └─ camera_link
```

---

## Scripts

| Script | Purpose |
|--------|---------|
| `deploy.sh` | rsync/ssh deploy to Jetson (`./scripts/deploy.sh jetson <IP>`) |
| `setup_ros2_humble.sh` | Fresh ROS2 Humble install |
| `setup_resume.sh` | Resume/fix partial ROS2 install |
| `fix_python.sh` | Fix Python path issues on Jetson |
| `nuke_ros.sh` | Remove ROS2 completely |
| `test_serial.sh` | Raw serial test for Cobra Flex without ROS |

---

## Hardware Summary

| Component | Spec | ROS Interface |
|-----------|------|---------------|
| Chassis | Waveshare Cobra Flex 4WD | Serial via cobra_driver |
| MCU | ESP32-S3 (motor controller) | `/dev/ttyUSB0` or `/dev/cobra_flex` |
| Compute | NVIDIA Jetson Orin Nano 8GB | — |
| LiDAR | RPLidar (2D) | `/scan` (driver not in this repo) |
| IMU | BNO055 9-DOF | `/imu/data` (disabled, future) |
| Camera | OAK-D Series 2 (depth) | Not yet integrated |
| Gamepad | Logitech F710 | `/dev/input/js0` via joy_node |
| Battery | 2× Zeee 3S 5200mAh LiPo | `/battery_voltage` |
