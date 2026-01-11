#!/bin/bash
# Phantom Omni FireWire - Host Setup Script
#
# Run this BEFORE starting Docker container (requires sudo)
# This script must be run on the HOST, not inside Docker
#
# Usage:
#   sudo ./setup_host.sh

set -e

echo "=== Phantom Omni Host Setup ==="

# Check if running as root
if [ "$EUID" -ne 0 ]; then
    echo "ERROR: Please run with sudo"
    echo "Usage: sudo $0"
    exit 1
fi

# Load FireWire kernel module
echo "[1/3] Loading FireWire kernel module..."
if lsmod | grep -q firewire_ohci; then
    echo "  firewire_ohci already loaded"
else
    modprobe firewire_ohci
    echo "  firewire_ohci loaded"
fi

# Wait for device detection
echo "[2/3] Waiting for device detection..."
sleep 1

# Set device permissions
echo "[3/3] Setting device permissions..."
if ls /dev/fw* 1>/dev/null 2>&1; then
    chmod 666 /dev/fw*
    echo "  Permissions set for:"
    ls -la /dev/fw*
else
    echo "  WARNING: No FireWire devices found (/dev/fw*)"
    echo "  Check:"
    echo "    - Phantom Omni power adapter (18V) connected"
    echo "    - FireWire cable in SECOND port"
    echo "    - Thunderbolt adapter connected"
    exit 1
fi

# Verify devices
echo ""
echo "=== Device Status ==="
if [ -d /sys/bus/firewire/devices ]; then
    ls /sys/bus/firewire/devices/
    echo ""
    echo "Expected: fw0 (adapter), fw1 (Phantom Omni)"
else
    echo "WARNING: /sys/bus/firewire/devices not found"
fi

echo ""
echo "=== Setup Complete ==="
echo "You can now start the Docker container:"
echo "  cd docker/ros2"
echo "  docker compose -f phantom/docker-compose.yml \\"
echo "    --env-file .env --env-file .env.local up"
