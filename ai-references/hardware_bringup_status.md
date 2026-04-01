# PepperByte Hardware Bringup Status

**Date:** 2026-04-01  
**Branch:** feat/mapping-app → next: IMU

---

## What Is Working

### Motors + Wheels
- Waveshare Cobra Flex ESP32 chassis running on `/dev/cobra_flex` (udev rule: `1a86:55d3`, QinHeng CH343 UART)
- cobra_driver sends T:13 JSON commands for velocity, T:12 for motor enable
- **Key fix:** DDSM400 motors require explicit enable (`T:12`) before accepting speed commands. If 12V power is applied after USB/ESP32 boot, the firmware's `setup()` enable sequence runs on unpowered motors and is lost. cobra_driver now sends `T:12` on every startup to re-enable regardless of power-on order.
- Keepalive sends `{"T":13,"X":0,"Z":0}` every 150ms (half of `cmd_vel_timeout_ms: 300ms`) to prevent the firmware 3s watchdog from stopping motors mid-drive

### Wheel Encoders + Odometry
- T:1001 feedback packets from ESP32 carry `odl`/`odr` (cumulative integer cm) and `v` (voltage × 100)
- cobra_driver computes differential-drive odometry from delta of cumulative values
- Publishes `/odom`, `/joint_states`, `/battery_voltage`, and odom→base_link TF
- joint_state_publisher removed — cobra_driver publishes `/joint_states` directly from encoder data

### RPLidar
- Model: Hardware Rev 18, Firmware 1.29 — **A2M12 or A3 variant** (40m range, DenseBoost scan mode)
- Serial: `/dev/rplidar` → `ttyUSB0` (Silicon Labs CP2102, `10c4:ea60`)
- Correct baud rate: **256000** (not 115200 — was wrong in original config, now fixed)
- Publishes `/scan` at 10Hz, frame_id `lidar_link`
- hardware_slam.launch.py updated to 256000 baud and `scan_mode: DenseBoost`

### Teleop
- `teleop_twist_keyboard` in a dedicated xterm window (launched via `ExecuteProcess` with full bash env)
- `xterm -e` + Node prefix approach was broken (no shell, raw executable had no ROS env); replaced with `ExecuteProcess` + `bash -c 'source ... && ros2 run ...'`
- Velocity flows: keyboard → `joy_vel` → twist_mux (priority 20) → `/cmd_vel` → cobra_driver

---

## Key Bugs Fixed This Session

| Bug | Root Cause | Fix |
|-----|-----------|-----|
| `/dev/cobra_flex` wrong port | udev rule pointed at ESP32 JTAG (`303a:1001`) instead of CH343 data UART (`1a86:55d3`) | Updated udev rule |
| Wheels not moving | DDSM400 motors need T:12 enable before accepting speed commands; firmware only does this during `setup()` — missed if 12V powered on after USB | cobra_driver sends T:12 on startup |
| Robot moved on launch | ESP32 RAM retains `ddsm_spd_1..4` across cobra_driver restarts; T:12 re-enabled motors at stale nonzero setpoints | Send `T:13 X:0 Z:0` immediately after T:12 |
| JSP stuck at startup | `joint_state_publisher` 2.4.0 ignores the `robot_description` ROS parameter; FastDDS TRANSIENT_LOCAL race also caused topic miss | Removed JSP entirely; cobra_driver publishes `/joint_states` |
| Odom not publishing on first packet | `updateOdometry()` had early `return` on first call | Removed early return; zero-delta odom published immediately |
| xterm teleop closed instantly | `prefix='xterm -e'` passes raw executable path without sourcing ROS env | Switched to `ExecuteProcess` with `bash -c 'source ... && ros2 run ...'` |
| RPLidar timeout | Wrong baud rate (115200 vs actual 256000) | Updated to 256000 baud |

---

## Next Tasks

### 1. IMU (BNO055)
- URDF has `imu_link` defined (`pepperbyte_description/urdf/imu.xacro`, BNO055)
- EKF config (`robot_localization_ekf.yaml`) already expects `/imu/data` — yaw rate + linear acceleration
- **Nothing is launching the IMU driver.** Need to:
  - Find/install a BNO055 ROS2 driver (e.g. `ros-humble-bno055` or `ros2-bno055`)
  - Add it to `hardware_slam.launch.py`
  - Confirm I2C address and bus on Jetson Orin Nano
  - Verify the EKF fusion matrix configuration

### 2. Web App → Potential Game Engine Migration
- Current: Flask + Flask-SocketIO backend, React 18 frontend (CDN, no build), Tailwind CSS
- ROS integration: rclpy node in background thread, subscribes to `/map`, `/odometry/filtered`, `/scan`, `/cmd_vel`
- Canvas-based 2D map rendering with robot pose, laser scan overlay, Nav2 goal clicking
- **Issue:** Flask/WebSocket introduces latency for real-time map rendering and control
- **Potential migration:** Replace web frontend with a lightweight game engine (Godot, Unity WebGL, or Pygame) for lower-latency rendering and control input. The ROS bridge layer (rosbridge_server or direct rclpy) would remain; only the frontend rendering/input changes.

### 3. SLAM Stack
- All packages present and building: `slam_toolbox`, `robot_localization`, `rf2o_laser_odometry`, `rplidar_ros`, `twist_mux`
- Needs IMU driver before EKF is fully functional (degrades gracefully without it)
- Known issues listed below

---

## SLAM Potential Issues & TODOs

| Item | File | Issue | Severity |
|------|------|-------|----------|
| `use_sim_time: true` hardcoded | `slam_toolbox_online_async.yaml`, `robot_localization_ekf.yaml` | Config says true but launch overrides to false for hardware. No runtime impact, but confusing. | Low |
| IMU driver missing | `hardware_slam.launch.py` | EKF expects `/imu/data` — no node publishes it on hardware. EKF runs on 2 sources only. | Medium |
| `rplidar_ros` not in package.xml | `pepperbyte_bringup/package.xml` | Missing `<exec_depend>rplidar_ros</exec_depend>`. Build succeeds but rosdep won't install it. | Medium |
| RPLidar baud 115200 | `hardware_slam.launch.py` | Was wrong — **now fixed to 256000** | Fixed |
| Aggressive SLAM node creation | `slam_toolbox_online_async.yaml` | New node every 5cm / 5° / 300ms. High CPU on Jetson; may need tuning under load. | Low |
| hardware_slam.launch.py uses `joy` + `teleop_twist_joy` | `hardware_slam.launch.py` | Joystick teleop — Logitech F710 not currently working on Jetson (xpad kernel module issue). Should add keyboard teleop fallback or make joy optional. | Medium |
| No map save directory configured | `pepperbyte_app` | App calls `ros2 run nav2_map_server map_saver_cli` without specifying output path. Maps save to current working directory of the Flask process. | Low |
| Hardcoded Flask secret key | `pepperbyte_app` | `app.secret_key = 'pepperbyte'` — not a security risk on LAN but should use env var. | Low |

---

## Architecture Reference (Current Working State)

```
/joy_vel  ──── twist_mux (priority 20) ──┐
/nav_vel  ──── twist_mux (priority 10) ──┴──→ /cmd_vel → cobra_driver → ESP32 → DDSM400 motors
                                                                ↑
                                                     T:1001 feedback (odl, odr, v)
                                                                ↓
                                          /odom + /joint_states + /battery_voltage

RPLidar A3 (/dev/rplidar, 256000 baud) → /scan (10Hz, lidar_link frame)

[IMU — NOT YET CONNECTED]              → /imu/data (expected by EKF)

/odom + /odom_rf2o + /imu/data → robot_localization EKF → /odometry/filtered
/scan + /odometry/filtered     → slam_toolbox           → /map
/map + /odometry/filtered      → Nav2                   → /nav_vel
/map                           → pepperbyte_app (Flask) → browser
```
