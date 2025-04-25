#!/bin/bash

# Exit on error and show commands
set -ex

# Get Docker PID with retries
for i in {1..15}; do
    DOCKERPID=$(docker inspect -f '{{.State.Pid}}' ros-core 2>/dev/null) && break
    sleep 1
done

# Check if a USB-CAN device is connected
if ! lsusb | grep -qE "PEAK System PCAN-USB|Kvaser AB Kvaser Leaf v3"; then
  echo "Warn! can0 interface was not found on the host and no USB-CAN device is connected!"
  exit 0  # But don't report it... We don't want the service to fail after all.
fi

# Trying to recover and reset the USB-CAN device
if ! command -v usbreset &>/dev/null; then
  echo "ERROR: usbreset command not found, automatic recovery not possible, please replug the USB-CAN device manually"
  echo "  For this feature to work, please install usbutils pkg on the host"
  exit 1
fi

# Attempt to reset the USB-CAN device
# Add device IDs for other USB-CAN devices here if needed
echo "Attempting to reset USB-CAN device..."
usbreset "PCAN-USB" || usbreset "Kvaser Leaf v3" || true

# Wait for up to 5 seconds for can0 interface to become available
for i in {1..5}; do
  if ip link show can0 &>/dev/null; then
    echo "can0 interface is now available."
    break
  fi
  echo "Waiting for can0 interface to become available... ($i/5)"
  sleep 1
done

# If can0 is still not available after 5 seconds, exit with an error
if ! ip link show can0 &>/dev/null; then
  echo "ERROR: can0 interface did not become available after 5 seconds."
  exit 1
fi

ip link set can0 type can bitrate 500000 restart-ms 100 || { echo "Failed to configure can0"; exit 1; }

if [ -z "$DOCKERPID" ]; then
    echo "ERROR: ros-core container not found!"
    exit 1
fi

# Create virtual CAN pair
ip link add vxcan0 type vxcan peer name vxcan1 netns "$DOCKERPID"

# Bring up interfaces
nsenter -t "$DOCKERPID" -n ip link set vxcan1 up
ip link set vxcan0 up

# Load can-gw module
modprobe can-gw

# Set up CAN gateway
cangw -A -s can0 -d vxcan0 -e
cangw -A -s vxcan0 -d can0 -e

ip link set can0 up || { echo "Failed to activate can0 in container"; exit 1; }

echo "CAN bridge setup completed successfully"