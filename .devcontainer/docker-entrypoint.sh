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

echo "--------------------------------------------------------------------"

rm -f /tmp/rexlaunch.pgid # remove old PGID file

# Copy host authorized keys for use with user rex
if [ -s /run/secrets/authorized_keys ]; then
    install -o rex -g "$(id -g rex)" -m 0600 /run/secrets/authorized_keys /home/rex/.ssh/authorized_keys
else
    echo "No authorized keys copied into the container"
fi

if service ssh start; then
    # Get the SSH port from the configuration file
    SSH_PORT_CONFIGURED=$(grep -oP '^Port\s+\K\d+' /etc/ssh/sshd_config)

    if [ -n "$SSH_PORT_CONFIGURED" ]; then
        echo "To connect to the container, use ssh rex@localhost -p ${SSH_PORT_CONFIGURED}"
    else
        echo "SSH service started, but could not determine port from /etc/ssh/sshd_config."
        echo "Please check the sshd_config file manually."
    fi
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