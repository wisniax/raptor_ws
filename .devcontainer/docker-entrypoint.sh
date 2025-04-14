#! /bin/bash

# This script will be called by the docker container when it starts
# and will start the REX ROS2 stuff

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