#!/bin/bash
# Copyright (c) 2026 Peppermint Robotics. All rights reserved.
# Deploy workspace to Jetson via rsync
JETSON_USER=${1:-jetson}
JETSON_IP=${2:-192.168.1.100}
rsync -avz --exclude='build/' --exclude='install/' --exclude='log/' --exclude='.git/' \
  ./ ${JETSON_USER}@${JETSON_IP}:~/pepperbyte_robot/
echo "Deployed. SSH in and run: cd ~/pepperbyte_robot && colcon build --symlink-install && source install/setup.bash"
