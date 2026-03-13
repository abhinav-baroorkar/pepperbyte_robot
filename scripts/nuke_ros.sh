#!/bin/bash
# Complete removal of ROS2 Humble + Python 3.10 from Ubuntu 24.04
# Restores the system to a clean state.
#
# Usage: sudo bash scripts/nuke_ros.sh
set -euo pipefail

if [ "$EUID" -ne 0 ]; then
  echo "ERROR: Run with sudo"
  exit 1
fi

echo "============================================"
echo "  Removing ROS2 Humble + Python 3.10"
echo "============================================"
echo ""

# ============================================================
# 1. Ensure python3 → 3.12 before removing anything
# ============================================================
echo "=== Step 1: Locking python3 → 3.12 ==="
update-alternatives --set python3 /usr/bin/python3.12 2>/dev/null || true
echo "  python3 → $(python3 --version)"

# ============================================================
# 2. Remove ALL ros-humble-* packages
# ============================================================
echo ""
echo "=== Step 2: Removing all ros-humble-* packages ==="
apt-get purge -y 'ros-humble-*' 2>&1 | tail -5

# ============================================================
# 3. Remove ROS2 apt source and key
# ============================================================
echo ""
echo "=== Step 3: Removing ROS2 apt sources ==="
rm -f /etc/apt/sources.list.d/ros2.sources
rm -f /etc/apt/sources.list.d/ros2.list
rm -f /usr/share/keyrings/ros-archive-keyring.gpg
echo "  Removed ros2.sources"

# ============================================================
# 4. Remove /opt/ros/humble entirely
# ============================================================
echo ""
echo "=== Step 4: Removing /opt/ros/ ==="
rm -rf /opt/ros/humble
rmdir /opt/ros 2>/dev/null || true
echo "  Removed /opt/ros/humble"

# ============================================================
# 5. Remove Python 3.10 and deadsnakes PPA
# ============================================================
echo ""
echo "=== Step 5: Removing Python 3.10 + deadsnakes PPA ==="
apt-get purge -y \
  python3.10 \
  python3.10-dev \
  python3.10-venv \
  python3.10-distutils \
  python3.10-minimal \
  libpython3.10 \
  libpython3.10-dev \
  libpython3.10-minimal \
  libpython3.10-stdlib \
  2>&1 | tail -5

# Remove deadsnakes PPA
add-apt-repository --remove -y ppa:deadsnakes/ppa 2>&1 | tail -3 || true
rm -f /etc/apt/sources.list.d/deadsnakes-ubuntu-ppa-noble.sources
rm -f /etc/apt/sources.list.d/deadsnakes-ubuntu-ppa-noble.list
echo "  Removed Python 3.10 + deadsnakes PPA"

# ============================================================
# 6. Remove update-alternatives entry for python3
# ============================================================
echo ""
echo "=== Step 6: Cleaning up update-alternatives ==="
update-alternatives --remove python3 /usr/bin/python3.10 2>/dev/null || true
update-alternatives --remove python3 /usr/bin/python3.12 2>/dev/null || true
# Restore the default symlink if alternatives broke it
if [ ! -e /usr/bin/python3 ]; then
  ln -sf python3.12 /usr/bin/python3
fi
echo "  Cleaned up"

# ============================================================
# 7. Reinstall core Python 3.12 packages (may have been damaged)
# ============================================================
echo ""
echo "=== Step 7: Reinstalling python3 system packages ==="
apt-get install --reinstall -y \
  python3 \
  python3-apt \
  python3-setuptools \
  python3-pkg-resources \
  python3-pip \
  2>&1 | tail -5

# ============================================================
# 8. Autoremove orphaned packages + clean apt cache
# ============================================================
echo ""
echo "=== Step 8: Cleaning up orphaned packages ==="
apt-get autoremove -y 2>&1 | tail -5
apt-get clean

# ============================================================
# 9. Remove ROS2 source line from bashrc
# ============================================================
echo ""
echo "=== Step 9: Cleaning up bashrc ==="
BASHRC_FILE="/home/${SUDO_USER:-$USER}/.bashrc"
if grep -q "source /opt/ros/humble/setup.bash" "$BASHRC_FILE" 2>/dev/null; then
  sed -i '/# ROS2 Humble/d' "$BASHRC_FILE"
  sed -i '\|source /opt/ros/humble/setup.bash|d' "$BASHRC_FILE"
  echo "  Removed ROS2 source line from $BASHRC_FILE"
else
  echo "  No ROS2 source line found in $BASHRC_FILE"
fi

# Also clean up any ROS env vars that may linger
sed -i '/^export ROS_DOMAIN_ID/d' "$BASHRC_FILE" 2>/dev/null || true

# ============================================================
# 10: Final verification
# ============================================================
echo ""
echo "=== Step 10: Verification ==="
echo "  python3:         $(python3 --version)"
echo -n "  apt_pkg:         "
python3 -c "import apt_pkg; print('OK')" 2>&1
echo -n "  setuptools:      "
python3 -c "import setuptools; print('OK')" 2>&1
echo -n "  ros2:            "
which ros2 2>/dev/null || echo "not found (expected)"
echo -n "  python3.10:      "
which python3.10 2>/dev/null || echo "not found (expected)"
echo -n "  /opt/ros/:       "
ls /opt/ros/ 2>/dev/null || echo "does not exist (expected)"

echo ""
echo "============================================"
echo "  Cleanup complete!"
echo "============================================"
echo ""
echo "Your system is back to stock Ubuntu 24.04."
echo "Open a new terminal to clear any stale env vars."
echo ""
echo "Your PepperByte workspace at ~/pepperbyte_robot"
echo "is untouched — it will build on the Jetson (22.04)"
echo "or after a clean ROS2 Humble install on 22.04."
