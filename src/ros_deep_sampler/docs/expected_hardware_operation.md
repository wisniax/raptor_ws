# Expected Hardware Operation

## 1. Control

The ROS package sends actuator commands to the MCU through CAN.

The current implementation sends **7 actuator commands** at a frequency of **20 Hz**:

- 3 × `VESC_COMMAND_SET_POS`
- 4 × `VESC_COMMAND_SET_RPM`

> **Note:** The vacuum clamp control will be changed in a future version.

### Actuator mapping

| Index | Actuator | CAN ID | Command | Value |
|---:|---|---|---|---|
| 0 | Platform | `sampler_platform` | `VESC_COMMAND_SET_POS` | `platform_cmd_` |
| 1 | Drill movement | `sampler_drill_mov` | `VESC_COMMAND_SET_POS` | `drill_cmd_` |
| 2 | Container | `sampler_container_a` | `VESC_COMMAND_SET_POS` | `container_cmd_` |
| 3 | Drill rotor | `sampler_drill` | `VESC_COMMAND_SET_RPM` | `rotor_cmd_` |
| 4 | Vacuum rotor | `sampler_vacuum_suction` | `VESC_COMMAND_SET_RPM` | `vacuum_rotor_cmd_` |
| 5 | Vacuum motor A | `sampler_vacuum_a` | `VESC_COMMAND_SET_RPM` | `brush_rotor_cmd_` |
| 6 | Vacuum motor B | `sampler_vacuum_b` | `VESC_COMMAND_SET_RPM` | `0.0` |

The corresponding ROS implementation is:

```cpp
cmd_.actuator_id[0] = RosCanConstants::VescIds::sampler_platform;
cmd_.actuator_id[1] = RosCanConstants::VescIds::sampler_drill_mov;
cmd_.actuator_id[2] = RosCanConstants::VescIds::sampler_container_a;
cmd_.actuator_id[3] = RosCanConstants::VescIds::sampler_drill;
cmd_.actuator_id[4] = RosCanConstants::VescIds::sampler_vacuum_suction;
cmd_.actuator_id[5] = RosCanConstants::VescIds::sampler_vacuum_a;
cmd_.actuator_id[6] = RosCanConstants::VescIds::sampler_vacuum_b;

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

sampler_can_cmd_pub_->publish(cmd_);

```

### Position-controlled actuators

The MCU should implement a position PID control loop for:

- Platform
- Drill movement
- Container

The expected units are:

- Platform: meters
- Drill: meters
- Container: radians

For the platform and drill:

- Positive position → upwards
- Negative position → downwards

The direction can be inverted in the MCU if required by the mechanical configuration.

For the container, the position is expressed in radians and its sign depends on the defined mechanical zero and rotation direction.

### Velocity-controlled actuators

The remaining actuators receive commands in RPM:

- Drill rotor
- Vacuum rotor
- Vacuum motors

The MCU is responsible for converting the requested RPM into the appropriate motor command.

## 2. Feedback

The MCU sends actuator feedback to the ROS package through CAN.

The feedback is expected to contain:

- Current position of the platform
- Current position of the drill
- Current position of the container
- Current velocity of all motors
- Rotor effort/current, if available
- Distance sensor readings 
- End-switch states, if available

The position and velocity values should use the same units as the commands:

| Quantity | Unit |
|:---|---|
| Platform position |	m |
| Drill position |	m |
| Container position |	rad |
| Motor velocity |	RPM |
| Rotor effort	| A (or another agreed unit)  |
| Distance sensor | m |

## 3. Calibration and Startup

At startup, the sampler performs a calibration procedure.

The three position-controlled axes are calibrated:

- Platform
- Drill
- Container

Each axis is commanded to move towards its maximum position until the corresponding end switch is activated.

When the end switch is reached:

- Stop the corresponding motor.
- Define this mechanical position as the calibration reference.
- Reset the actuator position to the defined zero position (0.0).
- Start tracking the position relative to this calibrated zero.

After calibration, the MCU must continuously maintain the current position and velocity of every actuator.

**Important**: The zero position must be established consistently after every calibration. ROS relies on these position values when generating and executing trajectories.

If an end switch is available, its state should also preferably be included in the feedback message.

## 4. Normal Operation

After calibration:

The MCU continuously receives commands from ROS.\
Position commands are handled by the MCU position controllers. \
RPM commands are converted to motor velocity commands.\
The MCU continuously measures actuator positions and velocities.\
Feedback is published to ROS at approximately 20 Hz.

The MCU should not rely on the ROS command frequency to maintain the motor state. The MCU should maintain the active control loop locally.

If no new command is received for a defined timeout period, the MCU should preferably enter a safe state (for example, stop the motors). \
The timeout value should be agreed between the ROS and MCU implementations.

## 5. Control Modes

The actuator control mode is determined by the command received from ROS.

The main modes are:

- Position control – VESC_COMMAND_SET_POS
- Velocity control – VESC_COMMAND_SET_RPM
- Manual/duty control – if VESC_COMMAND_SET_DUTY is used (to DO if required)

Only one command should be active for a given actuator at a time.

When switching between control modes, the MCU should properly reset or initialize the corresponding controller state (for example, PID integrator) to prevent unwanted motor movement.

## 6 Summary 

The MCU is responsible for:

- Motor control loops
- Position/velocity control
- Calibration
- End-switch handling
- Reading motor/sensor feedback
- Maintaining actuator state

The ROS hardware interface is responsible for:

- Translating ros2_control commands to CAN commands
- Translating CAN feedback into ROS joint states
- Providing actuator states to the ROS controllers
- Managing the communication between the ROS control system and the MCU