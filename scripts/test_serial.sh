#!/bin/bash
# Copyright (c) 2026 Peppermint Robotics. All rights reserved.
# Quick test: send a single feedback request to Cobra Flex and print the response
# Usage: ./scripts/test_serial.sh [/dev/ttyUSB0]

PORT=${1:-/dev/ttyUSB0}

if [ ! -e "$PORT" ]; then
    echo "ERROR: Serial port $PORT not found"
    echo "Available ports:"
    ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null || echo "  No USB serial ports detected"
    exit 1
fi

echo "Sending feedback request to Cobra Flex on $PORT..."
stty -F "$PORT" 115200 cs8 -cstopb -parenb raw -echo
echo '{"T":130}' > "$PORT"
sleep 0.5
timeout 2 cat "$PORT"
echo ""
echo "If you see a JSON response with T:1001, the connection is working."
