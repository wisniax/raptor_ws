#! /bin/bash

# This script will be called by the docker container when it starts
# and will start the REX ROS2 stuff

# Capture start time to brag about startup times later
start_time=`date +%s.%N`

# Stop script on errors
set -e

# Most important thing: logo ;)
echo "--------------------------------------------------------------------"
echo 'ooooooooo.         .o.       ooooooooo.   ooooooooooooo   .oooooo.   ooooooooo.    .oooooo..o  '
echo '`888   `Y88.      .888.      `888   `Y88. 8`   888   `8  d8P`  `Y8b  `888   `Y88. d8P`    `Y8  '
echo ' 888   .d88`     .8"888.      888   .d88`      888      888      888  888   .d88` Y88bo.       '
echo ' 888ooo88P`     .8` `888.     888ooo88P`       888      888      888  888ooo88P`   `"Y8888o.   '
echo ' 888`88b.      .88ooo8888.    888              888      888      888  888`88b.        `"Y88br  '
echo ' 888  `88b.   .8`     `888.   888              888      `88b    d88`  888  `88b.  oo     .d8P  '
echo 'o888o  o888o o88o     o8888o o888o            o888o      `Y8bood8P`  o888o  o888o 8""88888P`   '

echo "--------------------------------------------------------------------"
echo "Wakey wakey, eggs and bakey! Hello on rex startup!"
echo "--------------------------------------------------------------------"

echo "Loaded configuration:"
echo " - ROS_ENABLE_AUTOSTART:  ${ROS_ENABLE_AUTOSTART}"
echo " - ROS_BUILD_ON_STARTUP:  ${ROS_BUILD_ON_STARTUP}"
echo " - ROS_ENABLE_CAN_BRIDGE: ${ROS_ENABLE_CAN_BRIDGE}"

echo "--------------------------------------------------------------------"

rm -f /tmp/rexlaunch.pgid # remove old PGID file

if service ssh start; then
    echo "To connect to the container, use ssh rex@localhost -p 2122"
else
    echo "SSH service failed to start"
fi

echo "--------------------------------------------------------------------"

# Do some logs shenanigans
if [ -f /tmp/rex_launch.log ]; then
    rm -rf /tmp/rex_launch.log
    echo "Successfully removed old log file" "rex"
fi
touch /tmp/rex_launch.log
chown rex:1000 /tmp/rex_launch.log
chmod 664 /tmp/rex_launch.log

# Make raptor_ws directory writable
chmod g+rw -R /home/rex/raptor_ws
chmod g+rw -R /mnt/local

# Trigger and wait for can bridge creation on host's side
if [[ ${ROS_ENABLE_CAN_BRIDGE} == "1" ]]; then
    touch /mnt/local/.can_bridge_rex_waiting
    chown rex:1000 /mnt/local/.can_bridge_rex_waiting
    chmod 664 /mnt/local/.can_bridge_rex_waiting
    echo "rex is waiting for host system to create a network"

    # Total duration to check (seconds)
    canb_timeout=5
    # Interval between checks (seconds)
    canb_interval=0.5
    canb_end_time=$(( $(date +%s) + canb_timeout ))
    canb_interface_up=0

    echo "Rex: Waiting for vxcan1 to come up (max ${canb_timeout} seconds)..."

    while [[ $(date +%s) -lt $canb_end_time ]]; do
        sleep $canb_interval
        if [[ -d /sys/class/net/vxcan1 ]] && [[ $(cat /sys/class/net/vxcan1/operstate) == "up" ]]; then
            echo "Rex: vxcan1 is UP!"
            canb_interface_up=1
            break
        fi
    done

    if [[ $canb_interface_up -eq 0 ]]; then
        echo "REX ERROR: vxcan1 did not come up within ${canb_timeout} seconds - probably bad configuration. Please refer to README.md"
        rm -rf /mnt/local/.can_bridge_rex_waiting || true
    fi

    if [ -f /mnt/local/.can_bridge_rex_waiting ]; then
        rm -rf /mnt/local/.can_bridge_rex_waiting
        echo "REX WARN: vxcan1 setup service might fail - network probably is created but not bridged - please check the configuration"
    fi
else
    echo "Rex can bridge setup disabled - skipping"
fi

if [[ ${ROS_ENABLE_AUTOSTART} == "1" ]]; then
    echo "Rex autostart enabled - starting REX ROS2 ..."
    if service rex start; then
        : # do nothing on success
    else
        echo "Failed to start REX ROS2! Please make sure the package starts at all..."
    fi
else
    echo "Rex autostart disabled - skipping"
fi

echo "--------------------------------------------------------------------"
end_time=`date +%s.%N`
# runtime=$( echo "$end_time - $start_time" | bc -l )
runtime=$(printf '%.3f sec' "$(echo "scale=3;$end_time - $start_time" | bc)")
echo "Container startup sequence took: $runtime. Tailing log file..."
echo "--------------------------------------------------------------------"
tail -f -n 169 /tmp/rex_launch.log