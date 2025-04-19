#!/bin/bash

# Exit on error and show commands
set -ex

# Get Docker PID with retries
for i in {1..15}; do
    DOCKERPID=$(docker inspect -f '{{.State.Pid}}' ros-core 2>/dev/null) && break
    sleep 1
done

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

echo "CAN bridge setup completed successfully"