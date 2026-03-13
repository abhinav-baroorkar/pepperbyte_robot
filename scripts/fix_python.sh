#!/bin/bash
# Fix ROS2 Humble + Python on Ubuntu 24.04
#
# What went wrong: the earlier setup script set python3 → 3.10 which
# broke gnome-terminal, apt_pkg, setuptools, and other system tools.
#
# What this script does:
#   1. Ensures python3 → 3.12 (system default, never touch this again)
#   2. Reinstalls python3-setuptools to fix _distutils_hack error
#   3. Patches ROS2 Humble scripts to use python3.10 explicitly
#   4. Patches colcon to use python3.10
#   5. Verifies everything
#
# Usage: sudo bash scripts/fix_python.sh
set -euo pipefail

if [ "$EUID" -ne 0 ]; then
  echo "ERROR: Run with sudo"
  exit 1
fi

echo "============================================"
echo "  Fixing ROS2 Humble + Python on 24.04"
echo "============================================"
echo ""

# ============================================================
# Step 1: Lock python3 → 3.12 (system default)
# ============================================================
echo "=== Step 1/5: Ensuring python3 → 3.12 ==="
update-alternatives --set python3 /usr/bin/python3.12 2>/dev/null || true
echo "  python3 → $(python3 --version)"
echo ""

# ============================================================
# Step 2: Reinstall python3-setuptools to fix _distutils_hack
# ============================================================
echo "=== Step 2/5: Fixing python3-setuptools ==="
apt-get install --reinstall -y python3-setuptools 2>&1 | tail -3
# Verify the fix
if python3 -c "import setuptools" 2>/dev/null; then
  echo "  setuptools: OK"
else
  echo "  setuptools: still broken, trying harder..."
  apt-get install --reinstall -y python3-pkg-resources python3-setuptools 2>&1 | tail -3
fi
echo ""

# ============================================================
# Step 3: Patch ROS2 Humble scripts to use python3.10
#
# ROS2 Humble's Python .so files are compiled for cpython-310.
# They CANNOT load under python3.12. So every ROS2 script shebang
# must say python3.10, not python3.
# ============================================================
echo "=== Step 3/5: Patching ROS2 scripts → python3.10 ==="

PATCHED=0

# Patch scripts in /opt/ros/humble/bin/
for script in /opt/ros/humble/bin/*; do
  if [ -f "$script" ] && head -1 "$script" | grep -qE '#!.*python3(\s|$)'; then
    sed -i '1s|python3\b|python3.10|' "$script"
    PATCHED=$((PATCHED + 1))
  fi
done
echo "  /opt/ros/humble/bin/: patched $PATCHED scripts"

# Patch Python node executables in /opt/ros/humble/lib/
LIB_PATCHED=0
find /opt/ros/humble/lib -maxdepth 3 -type f -executable 2>/dev/null | while read -r script; do
  if head -1 "$script" 2>/dev/null | grep -qE '#!.*python3(\s|$)'; then
    sed -i '1s|python3\b|python3.10|' "$script"
  fi
done
echo "  /opt/ros/humble/lib/: scanned and patched"
echo ""

# ============================================================
# Step 4: Patch colcon and other build tools
# ============================================================
echo "=== Step 4/5: Patching colcon + build tools ==="

for script in \
  /usr/bin/colcon \
  /usr/bin/rosdep \
  /usr/bin/rosidl \
  /usr/bin/ament_* \
; do
  if [ -f "$script" ] && head -1 "$script" | grep -qE '#!.*python3(\s|$)'; then
    sed -i '1s|python3\b|python3.10|' "$script"
    echo "  Patched: $script"
  fi
done 2>/dev/null

# Also handle pip deps for python3.10
echo ""
echo "  Installing pip deps for python3.10..."
python3.10 -m pip install --quiet --break-system-packages \
  empy==3.3.4 \
  lark \
  catkin_pkg \
  setuptools \
  2>&1 | tail -3 || true
echo ""

# ============================================================
# Step 5: Verify everything
# ============================================================
echo "=== Step 5/5: Verification ==="

echo -n "  System python3:  "
python3 --version

echo -n "  python3.10:      "
python3.10 --version

echo -n "  apt_pkg import:  "
python3 -c "import apt_pkg; print('OK')" 2>&1

echo -n "  setuptools:      "
python3 -c "import setuptools; print('OK')" 2>&1

# Source ROS2 and test
export PYTHONPATH="/opt/ros/humble/lib/python3.10/site-packages:/opt/ros/humble/local/lib/python3.10/dist-packages${PYTHONPATH:+:$PYTHONPATH}"
export PATH="/opt/ros/humble/bin:$PATH"

echo -n "  ros2 CLI:        "
ros2 --version 2>&1 || echo "FAILED — see below"

echo -n "  rclpy import:    "
python3.10 -c "import rclpy; print('OK')" 2>&1

echo -n "  colcon:          "
if command -v colcon &>/dev/null; then
  colcon version-check 2>/dev/null | head -1 || echo "installed"
else
  echo "not found"
fi

echo ""
echo "============================================"
echo "  Fix complete!"
echo "============================================"
echo ""
echo "Open a NEW terminal, then run:"
echo "  source /opt/ros/humble/setup.bash"
echo "  ros2 --version"
echo "  python3 --version   # should be 3.12"
echo ""
echo "To build PepperByte:"
echo "  cd ~/pepperbyte_robot"
echo "  colcon build --symlink-install"
echo "  source install/setup.bash"
