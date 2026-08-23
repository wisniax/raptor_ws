# Can Bridge

The `can_bridge` package serves as the central communication bridge for the Rex 
robot, integrating the **ROS 2** system with the **CAN bus**. It is responsible 
for translating high-level control commands into the VESC protocol and aggregating
telemetry data from motors and sensors.

## System Architecture

The package utilizes a **modular architecture (Composable Nodes)**. 
The logic is distributed across eight independent components, providing process
isolation and enabling flexible thread configuration (for future multi-threading
implementations).

### Executive Components
*   **MotorControl**: Manages the wheel drive system (4x VESC drive motors, 4x Cubemars steering motors).
*   **ManipulatorControl**: Responsible for controlling the 6-axis robotic arm and the gripper.
*   **SamplerControl**: Handles the soil sampling mechanism.
*   **ConfigControl**: Manages the transmission of motor calibration frames.

### Status Components (Forwarders)
*   **VescStatusHandler**: Aggregates raw status frames from the CAN bus and publishes collective motor telemetry (RPM, current, temperature, position).
*   **BatteryInfoForwarder**: Forwards power supply and safety data from the onboard BMS (Battery Management System).
*   **SamplerStatusForwarder**: Forwards scientific measurement results (pH, weight, distance).
*   **StatusMessage**: Transmits ROS connection status and operation mode information to the electronics on the CAN bus. 

---

## Topic List

| Direction     | Topic Name | Message Type                            | Description                            |
|:--------------| :--- |:----------------------------------------|:---------------------------------------|
| **ROS > CAN** | `can_set_motor_vel` | `rex_interfaces/msg/Wheels`             | Set velocities and wheel turn angles.  |
| **ROS > CAN** | `can_manipulator_ctl` | `rex_interfaces/msg/ManipulatorControl` | Commands for arm axes and gripper.     |
| **ROS > CAN** | `can_calibration_motor_command`| `rex_interfaces/msg/VescMotorCommand`   | Wheel calibration commands.            |
| **ROS > CAN** | `mqtt_sampler_control`| `rex_interfaces/msg/SamplerControl`     | Sampling system control commands.      |
| **CAN > ROS** | `can_vesc_status` | `rex_interfaces/msg/VescStatus`         | Motor telemetry.                       |
| **CAN > ROS** | `can_battery_info` | `rex_interfaces/msg/BatteryInfo`        | Voltage, current, and BMS status.      |
| **CAN > ROS** | `can_sampler_status` | `rex_interfaces/msg/SamplerFeedback`    | Scientific sensor measurement results. |
| **Hardware**  | `can_raw_TX` / `RX` | `can_msgs/msg/Frame`                    | Raw physical bus interface.            |

---

## Safety Logic 

The bridge acts as the primary safety filter. All control frames are processed based on the bitmask of the `control_mode` field:

### Critical States
*   **ESTOP (RoverStatus)**: Immediate control cutoff for all nodes and transmission of a `0.0A` command to the drives.
*   **DRIVE_STOP (BatteryInfo)**: Activation of the physical "black mushroom" emergency button. Immediate control cutoff across all nodes.

### Mode Gating

Control commands are only passed to modules whose corresponding bits are currently
set in the mode mask (e.g., `ROBOTIC_ARM`). An exception is made for driving
frames, which are currently passed through regardless of the mode (workaround).

---

## Launching

The entire system is launched using a single launch file, which initializes 8 independent processes/components:

```bash
ros2 launch can_bridge can_bridge.yaml
