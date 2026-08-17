# CAN Bridge Communication

This document describes communication between the `sampler_hardware_interfaces` hardware interface and `can_bridge` using ROS 2 messages.

## CAN Command

**ROS message:** `sampler_motion_interfaces/msg/SamplerCanCmd`

```text
uint8[] actuator_id
uint8[] command_id
float32[] set_value
```

The arrays are indexed by actuator. The `actuator_id`, `command_id`, and `set_value` entries at the same index describe one CAN command.

## CAN Feedback

**ROS message:** `sampler_motion_interfaces/msg/SamplerFeedback`

```text
uint8[] actuator_id
float64[] positions
float64[] velocities
float64 rotor_effort
float64 platform_press
```

`positions` and `velocities` correspond to the actuators listed in `actuator_id`. `rotor_effort` and `platform_press` provide additional sampler feedback.

## Hardware Interface → CAN Bridge

When `sampler_mode` is enabled, the hardware interface fills the command message as follows:

```cpp
if (sampler_mode) {
    cmd_.command_id[0] = VESC_COMMAND_SET_POS;
    cmd_.command_id[1] = VESC_COMMAND_SET_POS;
    cmd_.command_id[2] = VESC_COMMAND_SET_POS;
    cmd_.command_id[3] = VESC_COMMAND_SET_RPM;
    cmd_.command_id[4] = VESC_COMMAND_SET_RPM;
    cmd_.command_id[5] = VESC_COMMAND_SET_RPM;
    cmd_.command_id[6] = VESC_COMMAND_SET_RPM;

    cmd_.set_value[0] = platform_cmd_;
    cmd_.set_value[1] = drill_cmd_;
    cmd_.set_value[2] = container_cmd_;
    cmd_.set_value[3] = rotor_cmd_;
    cmd_.set_value[4] = vacuum_rotor_cmd_;
    cmd_.set_value[5] = brush_rotor_cmd_;
    cmd_.set_value[6] = 0.0;
}
```

### Actuator mapping

| Index | Actuator | Command source | Command ID |
|---:|---|---|---|
| 0 | Platform | `platform_cmd_` | `VESC_COMMAND_SET_POS` |
| 1 | Drill | `drill_cmd_` | `VESC_COMMAND_SET_POS` |
| 2 | Container | `container_cmd_` | `VESC_COMMAND_SET_POS` |
| 3 | Rotor | `rotor_cmd_` | `VESC_COMMAND_SET_RPM` |
| 4 | Vacuum rotor | `vacuum_rotor_cmd_` | `VESC_COMMAND_SET_RPM` |
| 5 | Brush rotor | `brush_rotor_cmd_` | `VESC_COMMAND_SET_RPM` |
| 6 | Reserved / unused | `0.0` | `VESC_COMMAND_SET_RPM` |

## CAN Bridge → Hardware Interface

The CAN bridge publishes `SamplerFeedback`. The hardware interface stores the received actuator positions and velocities and exposes them through its `ros2_control` state interfaces.

```text
CAN bus
  ↓
can_bridge
  ↓
SamplerFeedback
  ↓
sampler_hardware_interfaces
  ↓
ros2_control state interfaces
  ↓
joint_state_broadcaster
  ↓
/joint_states
```

## Communication Flow

```text
Mission / Controllers
        ↓
ros2_control command interfaces
        ↓
sampler_hardware_interfaces
        ↓
SamplerCanCmd
        ↓
can_bridge
        ↓
CAN bus

CAN bus
  ↓
can_bridge
  ↓
SamplerFeedback
  ↓
sampler_hardware_interfaces
  ↓
ros2_control state interfaces
```
