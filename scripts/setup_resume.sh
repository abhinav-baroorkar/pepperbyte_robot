#!/bin/bash
# Resume ROS2 Humble install — fixed for Ubuntu 24.04
#
# ros-humble-desktop CANNOT install on 24.04 (needs libopencv4.5, libpcl1.12
# which don't exist on Noble). Instead we install ros-humble-ros-base +
# only the specific packages PepperByte actually needs.
#
# Usage: sudo bash scripts/setup_resume.sh
set -euo pipefail

if [ "$EUID" -ne 0 ]; then
  echo "ERROR: Run with sudo"
  exit 1
fi

log() { echo "[$(date +%H:%M:%S)] $*"; }

# ============================================================
# Step 1: Fix held packages / version conflicts
# ============================================================
log "=== Step 1: Fixing apt state ==="
apt-get update 2>&1 | tail -3
apt-get install -y --fix-broken 2>&1 | tail -3
# Upgrade libstdc++ and gcc-13-base to resolve the version mismatch
apt-get install -y gcc-13-base libstdc++-13-dev 2>&1 | tail -3
log "  Done"

# ============================================================
# Step 2: Install ros-humble-ros-base (no OpenCV/PCL deps)
# ============================================================
log "=== Step 2: Installing ros-humble-ros-base ==="
apt-get install -y \
  ros-humble-ros-base \
  2>&1 | tail -10
log "  Done"

# ============================================================
# Step 3: Install ros2cli + launch tools (from ros-core, not desktop)
# ============================================================
log "=== Step 3: Installing ros2 CLI tools ==="
apt-get install -y \
  ros-humble-ros2cli \
  ros-humble-ros2run \
  ros-humble-ros2launch \
  ros-humble-ros2node \
  ros-humble-ros2topic \
  ros-humble-ros2param \
  ros-humble-ros2pkg \
  ros-humble-ros2service \
  ros-humble-ros2bag \
  ros-humble-launch-xml \
  ros-humble-launch-yaml \
  2>&1 | tail -10
log "  Done"

# ============================================================
# Step 4: Install rviz2 (check if it can install — may fail on 24.04)
# ============================================================
log "=== Step 4: Installing rviz2 (if compatible) ==="
if apt-get install -y ros-humble-rviz2 2>&1 | tail -5; then
  log "  rviz2 installed"
else
  log "  rviz2 FAILED to install (OpenCV/Qt conflict on 24.04)"
  log "  You can view maps from a separate Ubuntu 22.04 machine via ROS_DOMAIN_ID"
fi

# ============================================================
# Step 5: Install colcon and build tools
# ============================================================
log "=== Step 5: Installing colcon + build tools ==="
apt-get install -y \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-vcstool \
  2>&1 | tail -5

if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  rosdep init 2>&1 || true
fi
log "  Done"

# ============================================================
# Step 6: Install PepperByte-specific packages
#         (install individually so one failure doesn't block all)
# ============================================================
log "=== Step 6: Installing PepperByte dependencies ==="

PKGS=(
  ros-humble-slam-toolbox
  ros-humble-robot-localization
  ros-humble-nav2-bt-navigator
  ros-humble-nav2-planner
  ros-humble-nav2-controller
  ros-humble-nav2-costmap-2d
  ros-humble-nav2-recoveries
  ros-humble-nav2-lifecycle-manager
  ros-humble-nav2-map-server
  ros-humble-nav2-regulated-pure-pursuit-controller
  ros-humble-nav2-navfn-planner
  ros-humble-robot-state-publisher
  ros-humble-joint-state-publisher
  ros-humble-xacro
  ros-humble-tf2
  ros-humble-tf2-ros
  ros-humble-tf2-geometry-msgs
  ros-humble-joy
  ros-humble-teleop-twist-joy
  ros-humble-twist-mux
  ros-humble-rplidar-ros
)

FAILED_PKGS=()
for pkg in "${PKGS[@]}"; do
  if apt-get install -y "$pkg" 2>&1 | tail -1 | grep -q "is already the newest"; then
    : # already installed, skip
  elif ! dpkg -l "$pkg" 2>/dev/null | grep -q '^ii'; then
    log "  FAILED: $pkg"
    FAILED_PKGS+=("$pkg")
  fi
done

if [ ${#FAILED_PKGS[@]} -gt 0 ]; then
  log "  WARNING: These packages failed to install: ${FAILED_PKGS[*]}"
else
  log "  All packages installed successfully"
fi

# pip deps for python3.10
python3.10 -m pip install --break-system-packages --quiet \
  empy==3.3.4 lark catkin_pkg 2>&1 || true
log "  Done"

# ============================================================
# Step 7: Patch ROS2 + colcon shebangs → python3.10
# ============================================================
log "=== Step 7: Patching shebangs → python3.10 ==="

PATCHED=0
for script in /opt/ros/humble/bin/*; do
  if [ -f "$script" ] && head -1 "$script" | grep -qE '#!.*python3(\s|$)'; then
    sed -i '1s|python3\b|python3.10|' "$script"
    PATCHED=$((PATCHED + 1))
  fi
done
log "  /opt/ros/humble/bin/: $PATCHED scripts"

LIB_PATCHED=0
while IFS= read -r script; do
  if head -1 "$script" 2>/dev/null | grep -qE '#!.*python3(\s|$)'; then
    sed -i '1s|python3\b|python3.10|' "$script"
    LIB_PATCHED=$((LIB_PATCHED + 1))
  fi
done < <(find /opt/ros/humble/lib -maxdepth 3 -type f -executable 2>/dev/null)
log "  /opt/ros/humble/lib/: $LIB_PATCHED scripts"

for script in /usr/bin/colcon /usr/bin/rosdep /usr/bin/ament_*; do
  if [ -f "$script" ] && head -1 "$script" | grep -qE '#!.*python3(\s|$)'; then
    sed -i '1s|python3\b|python3.10|' "$script"
    log "  Patched: $script"
  fi
done 2>/dev/null

log "  Done"

# ============================================================
# Step 8: Verify
# ============================================================
log "=== Step 8: Verification ==="

update-alternatives --set python3 /usr/bin/python3.12 2>/dev/null || true
source /opt/ros/humble/setup.bash

log "  python3:      $(python3 --version)"
log "  python3.10:   $(python3.10 --version)"
log "  ROS_DISTRO:   ${ROS_DISTRO:-not set}"

if command -v ros2 &>/dev/null; then
  log "  ros2:         $(ros2 --version 2>/dev/null || echo 'present')"
else
  log "  ros2:         NOT FOUND"
fi

log "  rclpy:        $(python3.10 -c 'import rclpy; print("OK")' 2>&1)"

if command -v colcon &>/dev/null; then
  log "  colcon:       present"
else
  log "  colcon:       NOT FOUND"
fi

# Add sourcing to bashrc if not already there
BASHRC_FILE="/home/${SUDO_USER:-$USER}/.bashrc"
if ! grep -q "source /opt/ros/humble/setup.bash" "$BASHRC_FILE" 2>/dev/null; then
  printf '\n# ROS2 Humble\nsource /opt/ros/humble/setup.bash\n' >> "$BASHRC_FILE"
  log "  Added ROS2 source to $BASHRC_FILE"
else
  log "  ROS2 source already in $BASHRC_FILE"
fi

# rosdep update as user
if [ -n "${SUDO_USER:-}" ]; then
  su - "$SUDO_USER" -c "rosdep update" 2>&1 | tail -3 || true
fi

log ""
log "============================================"
log "  ROS2 Humble setup complete!"
log "============================================"
log ""
log "Open a NEW terminal, then:"
log "  ros2 --version"
log "  cd ~/pepperbyte_robot && colcon build --symlink-install"
