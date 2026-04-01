#!/bin/bash
# Copyright (c) 2026 Peppermint Robotics. All rights reserved.
# Run this once on the Jetson after cloning/pulling the repo.

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(dirname "$SCRIPT_DIR")"

echo "======================================================"
echo "  PepperByte Jetson Setup"
echo "======================================================"

# Install ROS2 dependencies
echo ""
echo "[1/4] Installing ROS2 packages..."
sudo apt-get update
sudo apt-get install -y \
  ros-humble-joy \
  ros-humble-teleop-twist-joy \
  ros-humble-twist-mux \
  ros-humble-robot-localization \
  ros-humble-slam-toolbox \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-rplidar-ros \
  ros-humble-xacro \
  ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher

# Install udev rules
echo ""
echo "[2/4] Installing udev rules..."
sudo cp "$REPO_DIR/src/cobra_driver/config/99-cobra-flex.rules" /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger

# Initialize submodules (rf2o_laser_odometry)
echo ""
echo "[3/4] Initializing git submodules..."
cd "$REPO_DIR"
git submodule update --init --recursive

# Build workspace
echo ""
echo "[4/4] Building workspace..."
cd "$REPO_DIR"
source /opt/ros/humble/setup.bash
colcon build --symlink-install

echo ""
echo "======================================================"
echo "  Setup complete!"
echo "======================================================"
echo ""
echo "Next steps:"
echo "  1. Plug in ESP32 USB cable"
echo "  2. Check:  ls /dev/cobra_flex"
echo "  3. Run:    source install/setup.bash"
echo "  4. Run:    ros2 launch pepperbyte_bringup hardware_teleop.launch.py"
echo ""
