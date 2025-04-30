#/bin/sh
# create a new session (so that all launched nodes share the same PGID, and thus can be bulk-terminated)
setsid /bin/sh -c '. install/setup.sh && export ROS_DOMAIN_ID=1 && ros2 launch master-launch.yaml >> /tmp/rex_launch.log' &

# record the process group ID (PGID)
PGID=$!
echo "Launched ROS2 master-launch in process group $PGID"

# save PGID to a file (so that later we know what to kill)
PIDFILE="/tmp/rexlaunch.pgid"
echo $PGID > $PIDFILE

