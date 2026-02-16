# Motor calibration

ROS node which allows for Cubemars motor calibration.<br />
Calibration can be performed in app E-STOP mode. The black mushroom cannot be pushed in.

[[Cubemars manual](https://www.cubemars.com/images/file/20240611/1718085712815162.pdf)] [[libVescCan](https://github.com/AlvaroBajceps/libVescCan/)]

## Table of Contents
- [Frame Format](#frame-format)
- [Modes](#modes)
    - [SetVelocity](#setvelocity)
    - [SetPos](#setpos)
    - [Hold](#hold)
    - [Nothing](#nothing)
- [Parameters](#parameters)
- [Actions](#actions)
    - [STOP](#action_type_stop-0)
    - [RETURN_TO_ORIGIN](#action_type_return_to_origin-1)
    - [CONFIRM](#action_type_confirm-2)
    - [CANCEL](#action_type_cancel-3)
    - [OFFSET](#action_type_offset-4)
    - [SET_VELOCITY](#action_type_set_velocity-5)
- [FAQ](#faq)


## Frame format

```
# action_type enum
uint8 ACTION_TYPE_STOP = 0
uint8 ACTION_TYPE_RETURN_TO_ORIGIN = 1
uint8 ACTION_TYPE_CONFIRM = 2
uint8 ACTION_TYPE_CANCEL = 3
uint8 ACTION_TYPE_OFFSET = 4
uint8 ACTION_TYPE_SET_VELOCITY = 5

std_msgs/Header header
uint8 vesc_id
uint8 action_type
float32 value
```

## Modes

The calibration node is in one four modes at all times.

### SetVelocity
Continously sends VESC `SET_RPM` frames over CAN.<br />
A timeout is set - if there is `SET_VELOCITY` frame from the app for a specified amount of time, mode is changed to `Hold`<br />
During this mode, only `CANCEL`, `STOP` and `SET_VELOCITY` frames are accepted.

### SetPos
Continously sends VESC `SET_POS` frames over CAN.<br />
Stops when ERPM = 0 and position is almost at target (see `stop_tolerance` param), it then switches to `Hold`.<br />
During this mode, only `CANCEL`, `STOP` frames are accepted.

### Hold
Actively holds the motor's position to prevent any drift (using VESC `SET_POS` frames).


### Nothing
Idle mode, does nothing. No frames are sent over CAN.

## Parameters

Parameter                            | Type  | Unit | Description
-------------------------------------|-------|------|-------------------------------------------------------------------------------------------
`max_speed`              | float | ERPM | Max speed for SetVelocity mode.[0]
`max_offset_shift`       | float | deg  | Max value for a single `Offset` command.[0]
`max_velocity_shift`     | float | deg  | Max rotation away from origin for SetPos mode[1].
`outdated_duration_s`    | float | s    | Time after which a `VescStatus` frame is considered outdated.
`speed_timeout_ms`       | float | ms   | Timeout for SetVelocity. If no frame comes from the app in this time, the motor is stopped
`message_send_period_ms` | float | ms   | Period at which to send VESC frames.
`stop_condition`         | int   | -    | Which variant of the SetPos stop condition to use
`stop_tolerance`         | int   | deg  | Position tolerance to decide when SetPos can be stopped
`log_setpos_diff`        | int   | 0/1  | Whether to log current_position<->target_position during SetPos
`use_schedule_hold`      | int   | 0/1  | Whether to delay holding until next received VESC status (to prevent jerk), see [FAQ](#faq)

[0] - values outside of range are clamped<br />
[1] - to rotate more, set origin and try again
<!-- 
It's possible to change parameters while the node is running. For instance:
```
ros2 param set calibrate_axis max_speed 3000.0
``` -->

## Actions

Each frame has to have `vesc_id` set to an ID of a motor.

### ACTION_TYPE_STOP [0]
Stops the motor and switches to `Hold`.

### ACTION_TYPE_RETURN_TO_ORIGIN [1]
Moves the motor to its origin position using `SetPos` mode.

### ACTION_TYPE_CONFIRM [2]
Sets the origin of a motor at its current position.

### ACTION_TYPE_CANCEL [3]
Stops the motor, doesn't hold its position. Use this to abort calibration.

### ACTION_TYPE_OFFSET [4]
Rotates the motor by `value` degrees. Cannot rotate more (at once) than specified in `max_offset_shift` param.
After reaching the target position switches to `Hold`

### ACTION_TYPE_SET_VELOCITY [5]
Sets the speed at which to rotate the motor.
If `0.0` is set, the motor stops and switches to `Hold`.
Cannot rotate more (in total) than specified in `max_velocity_shift` param.
If you need to rotate more, set origin and repeat.

## FAQ

- What happens if motor 2 starts being calibrated when motor 1 is still being calibrated?
    - Only one motor can be calibrated at a time. In this case, motor 1 would be stopped (without hold) and calibration of motor 2 would continue as normal
- What is hold scheduling?
    - Normally, when the motor is rotated by velocity and then stopped, it holds at the last known position. It may cause the motor to jerk backwards before stopping. When hold scheduling is enabled, the motor is first stopped, and held only when a new position is received.