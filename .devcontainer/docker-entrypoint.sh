#! /bin/bash

# This script will be called by the docker container when it starts
# and will start the REX ROS2 stuff
echo "--------------------------------------------------------------------"
echo "Wakey wakey, eggs and bakey! Hello on rex startup!"
echo "--------------------------------------------------------------------"
rm -f /tmp/rexlaunch.pgid

service ssh start
echo "--------------------------------------------------------------------"

# Do some logs shenanigans
if [ -f /tmp/rex_launch.log ]; then
    rm -rf /tmp/rex_launch.log
    echo "Successfully removed old log file" "rex"
fi
touch /tmp/rex_launch.log
chown rex:1000 /tmp/rex_launch.log
chmod 664 /tmp/rex_launch.log

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
echo "Container startup sequence complete. Tailing log file..."
echo "--------------------------------------------------------------------"
tail -f -n 169 /tmp/rex_launch.log