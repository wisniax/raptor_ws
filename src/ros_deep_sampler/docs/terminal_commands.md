# Sampler CLI

`sampler_cli.py` is a command-line tool for sending commands to the sampler through ROS 2.

The script publishes `sampler_motion_interfaces/msg/SamplerMissionCmd` messages to:

```text
/MQTT/MissionCommand
```

Run the script with:

```
ros2 run ros_deep_sampler sampler_cli.py <command> [<value> ...]
```

```
ros2 run ros_deep_sampler sampler_cli.py start
ros2 run ros_deep_sampler sampler_cli.py stop
ros2 run ros_deep_sampler sampler_cli.py abort
ros2 run ros_deep_sampler sampler_cli.py restart
ros2 run ros_deep_sampler sampler_cli.py calibrate
```

## Manual commands

Manual commands use the following format:

`ros2 run ros_deep_sampler sampler_cli.py platform -0.3`

Multiple actuators can be commanded at the same time:

```
ros2 run ros_deep_sampler sampler_cli.py \
    platform -0.3 \
    drill -0.2 \
    drill_rotor 3.0 \
    vacuum 5.0 \
    brush 2.0 \
    open_vacuum 1
```

## Important

When only one manual command is specified, only the corresponding field is explicitly set by the CLI. Other fields retain the default value defined by the ROS message.

## Command format summary
```
Mission:
    start
    stop
    abort
    restart
    restart_deep
    restart_surface
    calibrate

Manual:
    platform <value>
    drill <value>
    container <value>
    drill_rotor <value>
    vacuum <value>
    brush <value>
    open_vacuum <0|1>
```