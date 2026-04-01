#!/bin/bash
# Copyright (c) 2026 Peppermint Robotics. All rights reserved.
# Installs udev rules for PepperByte hardware (ESP32 + RPLidar).

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(dirname "$SCRIPT_DIR")"

echo "Installing PepperByte udev rules..."
sudo cp "$REPO_DIR/src/cobra_driver/config/99-cobra-flex.rules" /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
echo "Done. Plug in the ESP32 USB cable and check /dev/cobra_flex exists."
