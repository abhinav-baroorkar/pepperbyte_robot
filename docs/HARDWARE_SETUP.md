# PepperByte Hardware Setup

## Prerequisites
- NVIDIA Jetson Orin Nano 8GB with Ubuntu 22.04
- ROS2 Humble installed (`source /opt/ros/humble/setup.bash`)
- Waveshare Cobra Flex 4WD chassis with ESP32-S3
- RPLidar 2D scanner
- Logitech F710 gamepad (XInput mode — switch on back set to X)

## Initial Setup (run once)

```bash
# Clone the repo on the Jetson
git clone <repo-url> ~/pepperbyte_robot
cd ~/pepperbyte_robot

# Run the setup script — installs deps, udev rules, builds workspace
bash scripts/jetson_setup.sh
```

## Verify Hardware

```bash
# 1. Plug in ESP32 USB cable
ls -la /dev/cobra_flex
# Should show a symlink to /dev/ttyUSBx

# 2. Plug in RPLidar USB cable
ls -la /dev/rplidar
# If no udev rule for RPLidar yet, it will be at /dev/ttyUSB1

# 3. Plug in Logitech F710 USB dongle
ls -la /dev/input/js0
# Should show the joystick device
```

## Power Wiring

```
Battery EC5 (+) ──┬──> Chassis barrel jack (motor power, 11.1–12.6V)
                  └──> DC-DC buck converter ──> Jetson USB-C (5V 4A)
Battery EC5 (-) ──> common ground
```

**WARNING:** Always connect USB cables to Jetson BEFORE connecting the battery.
Use a 10A fuse on the positive line between battery and chassis.

## Connection Order (Every Session)

1. Check battery voltage with LiPo checker (must be >10.5V / >3.5V per cell)
2. Connect USB from ESP32 to Jetson
3. Connect USB from RPLidar to Jetson
4. SSH into Jetson or connect monitor
5. Verify `/dev/cobra_flex` exists
6. Connect battery barrel jack to chassis
7. Launch ROS2

**Disconnect order (reverse):**
1. Stop all ROS2 nodes (Ctrl+C in launch terminal)
2. Disconnect battery
3. Disconnect USB cables

## Launch Commands (Hardware)

```bash
cd ~/pepperbyte_robot
source install/setup.bash

# Teleop only — joystick driving, no SLAM (hardware checkout)
ros2 launch pepperbyte_bringup hardware_teleop.launch.py

# SLAM mapping — drive with joystick while building the map
ros2 launch pepperbyte_bringup hardware_slam.launch.py
```

## Launch Commands (Simulation)

```bash
cd ~/pepperbyte_robot
source install/setup.bash

# Gazebo teleop
ros2 launch pepperbyte_bringup gazebo_teleop.launch.py

# Gazebo SLAM
ros2 launch pepperbyte_bringup gazebo_slam.launch.py

# Gazebo Nav2 (autonomous navigation)
ros2 launch pepperbyte_bringup gazebo_nav.launch.py
```

## Verification Commands

```bash
# Check odometry is publishing
ros2 topic echo /odom --once

# Check battery voltage
ros2 topic echo /battery_voltage --once

# Check LiDAR is scanning
ros2 topic hz /scan

# Check TF tree is complete
ros2 run tf2_tools view_frames
```

## Saving Maps

```bash
mkdir -p ~/maps
ros2 run nav2_map_server map_saver_cli -f ~/maps/site_name
```

## Joystick Controls (Logitech F710)

| Control | Button/Axis | Action |
|---------|-------------|--------|
| Drive forward/back | Left stick vertical | Linear velocity |
| Turn left/right | Left stick horizontal | Angular velocity |
| Enable driving | Hold LB | Must hold to drive |
| Turbo mode | Hold RB | 0.5 m/s / 1.5 rad/s |
| Normal speed | — | 0.3 m/s / 1.0 rad/s |

## Troubleshooting

**`/dev/cobra_flex` not found:**
```bash
bash scripts/install_udev_rules.sh
# Unplug and replug the ESP32 USB cable
```

**Serial port permission denied:**
```bash
sudo chmod 666 /dev/cobra_flex
# Or add user to dialout group:
sudo usermod -aG dialout $USER
# Log out and back in
```

**No LiDAR data on `/scan`:**
- Check `ls /dev/ttyUSB*` — RPLidar may be on a different port
- Update `serial_port` parameter in the launch file if needed

**Robot doesn't move with joystick:**
- Ensure F710 is in XInput mode (switch on back = X)
- Press and hold LB while pushing the left stick
- Check: `ros2 topic echo /joy` — verify button presses appear
