#!/bin/bash
# Copyright (c) 2026 Peppermint Robotics. All rights reserved.
#
# ROS2 Humble setup on Ubuntu 24.04 (Noble)
#
# IMPORTANT: Does NOT change the system python3 (stays 3.12).
# Instead patches ROS2 scripts to use python3.10 explicitly.
#
# Usage: sudo bash scripts/setup_ros2_humble.sh
#
set -euo pipefail

if [ "$EUID" -ne 0 ]; then
  echo "ERROR: This script must be run as root (use sudo)"
  exit 1
fi

LOGFILE="/tmp/ros2_humble_setup_$(date +%Y%m%d_%H%M%S).log"
echo "Logging to $LOGFILE"

log() {
  echo "[$(date +%H:%M:%S)] $*" | tee -a "$LOGFILE"
}

# ============================================================
# Step 1: Ensure python3 → 3.12 (NEVER change this)
# ============================================================
log "=== Step 1/7: Locking python3 → 3.12 ==="
update-alternatives --set python3 /usr/bin/python3.12 2>/dev/null || true
log "  python3 → $(python3 --version)"
log ""

# ============================================================
# Step 2: Install Python 3.10 side-by-side (for ROS2 only)
# ============================================================
log "=== Step 2/7: Installing Python 3.10 (side-by-side) ==="

add-apt-repository -y ppa:deadsnakes/ppa 2>&1 | tee -a "$LOGFILE"
apt-get update 2>&1 | tee -a "$LOGFILE"

apt-get install -y \
  python3.10 \
  python3.10-dev \
  python3.10-venv \
  python3.10-distutils \
  libpython3.10-dev \
  2>&1 | tee -a "$LOGFILE"

log "  python3.10 → $(python3.10 --version)"

# Install pip for python3.10
python3.10 -m ensurepip --upgrade 2>&1 | tee -a "$LOGFILE" || true
python3.10 -m pip install --break-system-packages --upgrade pip setuptools 2>&1 | tee -a "$LOGFILE" || true
log ""

# ============================================================
# Step 3: Install ROS2 Humble desktop
# ============================================================
log "=== Step 3/7: Installing ROS2 Humble desktop ==="

apt-get install -y \
  ros-humble-desktop \
  ros-humble-ros-base \
  2>&1 | tee -a "$LOGFILE"

log "  ros-humble-desktop installed"
log ""

# ============================================================
# Step 4: Install colcon and build tools
# ============================================================
log "=== Step 4/7: Installing colcon and build tools ==="

apt-get install -y \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-vcstool \
  2>&1 | tee -a "$LOGFILE"

if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  rosdep init 2>&1 | tee -a "$LOGFILE" || true
fi

log "  Build tools installed"
log ""

# ============================================================
# Step 5: Install PepperByte-specific dependencies
# ============================================================
log "=== Step 5/7: Installing PepperByte ROS2 dependencies ==="

apt-get install -y \
  ros-humble-slam-toolbox \
  ros-humble-robot-localization \
  ros-humble-nav2-bringup \
  ros-humble-nav2-bt-navigator \
  ros-humble-nav2-planner \
  ros-humble-nav2-controller \
  ros-humble-nav2-costmap-2d \
  ros-humble-nav2-recoveries \
  ros-humble-nav2-lifecycle-manager \
  ros-humble-nav2-map-server \
  ros-humble-nav2-regulated-pure-pursuit-controller \
  ros-humble-nav2-navfn-planner \
  ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher \
  ros-humble-xacro \
  ros-humble-tf2 \
  ros-humble-tf2-ros \
  ros-humble-tf2-geometry-msgs \
  ros-humble-joy \
  ros-humble-teleop-twist-joy \
  ros-humble-twist-mux \
  ros-humble-rplidar-ros \
  2>&1 | tee -a "$LOGFILE"

# pip deps for python3.10 that ROS2 build tools need
python3.10 -m pip install --break-system-packages --quiet \
  empy==3.3.4 lark catkin_pkg 2>&1 || true

log "  PepperByte dependencies installed"
log ""

# ============================================================
# Step 6: Patch ALL ROS2 + colcon scripts → python3.10
#
# ROS2 Humble .so modules are cpython-310 ABI.
# Scripts ship with #!/usr/bin/env python3 which resolves to
# python3.12 on Noble — that can't load cpython-310 .so files.
# Fix: rewrite shebangs to python3.10.
# ============================================================
log "=== Step 6/7: Patching ROS2 + tool shebangs → python3.10 ==="

PATCHED=0

# Patch /opt/ros/humble/bin/
for script in /opt/ros/humble/bin/*; do
  if [ -f "$script" ] && head -1 "$script" | grep -qE '#!.*python3(\s|$)'; then
    sed -i '1s|python3\b|python3.10|' "$script"
    PATCHED=$((PATCHED + 1))
  fi
done
log "  /opt/ros/humble/bin/: $PATCHED scripts patched"

# Patch Python node executables in /opt/ros/humble/lib/
LIB_PATCHED=0
while IFS= read -r script; do
  if head -1 "$script" 2>/dev/null | grep -qE '#!.*python3(\s|$)'; then
    sed -i '1s|python3\b|python3.10|' "$script"
    LIB_PATCHED=$((LIB_PATCHED + 1))
  fi
done < <(find /opt/ros/humble/lib -maxdepth 3 -type f -executable 2>/dev/null)
log "  /opt/ros/humble/lib/: $LIB_PATCHED scripts patched"

# Patch system-installed tools: colcon, rosdep, ament_*
for script in /usr/bin/colcon /usr/bin/rosdep /usr/bin/ament_*; do
  if [ -f "$script" ] && head -1 "$script" | grep -qE '#!.*python3(\s|$)'; then
    sed -i '1s|python3\b|python3.10|' "$script"
    log "  Patched: $script"
  fi
done 2>/dev/null

log ""

# ============================================================
# Step 7: Verify and configure shell
# ============================================================
log "=== Step 7/7: Verification ==="

# Re-confirm python3 is still 3.12 (paranoia check)
update-alternatives --set python3 /usr/bin/python3.12 2>/dev/null || true

# Source ROS2
source /opt/ros/humble/setup.bash

log "  System python3:  $(python3 --version)"
log "  ROS2 python:     $(python3.10 --version)"
log "  ROS_DISTRO:      ${ROS_DISTRO:-not set}"

if command -v ros2 &>/dev/null; then
  log "  ros2 CLI:        $(ros2 --version 2>/dev/null || echo 'present')"
else
  log "  ros2 CLI:        NOT FOUND"
fi

if command -v colcon &>/dev/null; then
  log "  colcon:          present"
else
  log "  colcon:          NOT FOUND"
fi

log "  rclpy import:    $(python3.10 -c 'import rclpy; print("OK")' 2>&1)"

# Add sourcing to bashrc
BASHRC_FILE="/home/${SUDO_USER:-$USER}/.bashrc"
if ! grep -q "source /opt/ros/humble/setup.bash" "$BASHRC_FILE" 2>/dev/null; then
  cat >> "$BASHRC_FILE" << 'BASHEOF'

# ROS2 Humble
source /opt/ros/humble/setup.bash
BASHEOF
  log "  Added ROS2 source to $BASHRC_FILE"
else
  log "  ROS2 source already in $BASHRC_FILE"
fi

# rosdep update as user
if [ -n "${SUDO_USER:-}" ]; then
  su - "$SUDO_USER" -c "rosdep update" 2>&1 | tee -a "$LOGFILE" || true
fi

log ""
log "============================================"
log "  ROS2 Humble setup complete!"
log "============================================"
log ""
log "Open a NEW terminal, then:"
log "  ros2 --version"
log "  python3 --version    # should say 3.12"
log "  cd ~/pepperbyte_robot && colcon build --symlink-install"
log ""
log "Full log: $LOGFILE"
