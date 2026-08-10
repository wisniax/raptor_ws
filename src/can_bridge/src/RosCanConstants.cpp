#include "can_bridge/RosCanConstants.hpp"

const std::string RosCanConstants::RosTopics::can_raw_RX = "/CAN/RX/raw";
const std::string RosCanConstants::RosTopics::can_raw_TX = "/CAN/TX/raw";
const std::string RosCanConstants::RosTopics::can_set_motor_vel = "/CAN/TX/set_motor_vel";
const std::string RosCanConstants::RosTopics::can_get_motor_vel = "/CAN/RX/real_motor_vel";
const std::string RosCanConstants::RosTopics::can_get_motor_current = "/CAN/RX/real_motor_curr";
const std::string RosCanConstants::RosTopics::can_stm_errors = "/CAN/RX/stm_errors";
const std::string RosCanConstants::RosTopics::can_stm_init = "/CAN/TX/stm_init";
const std::string RosCanConstants::RosTopics::can_vesc_status = "/CAN/RX/vesc_status";
const std::string RosCanConstants::RosTopics::can_manipulator_ctl = "/CAN/TX/manipulator_ctl";
const std::string RosCanConstants::RosTopics::can_sampler_status = "/CAN/RX/sampler_status";
const std::string RosCanConstants::RosTopics::can_battery_info = "/CAN/RX/battery_info";
const std::string RosCanConstants::RosTopics::can_calibration_motor_command = "/CAN/TX/calibration_motor_command";

const std::string RosCanConstants::RosTopics::mqtt_rover_status = "/MQTT/RoverStatus";
const std::string RosCanConstants::RosTopics::mqtt_sampler_control = "/MQTT/SamplerControl";
const std::string RosCanConstants::RosTopics::mqtt_calibrate_axis = "/MQTT/CalibrateAxis";

const uint8_t RosCanConstants::VescIds::ros_can_host = 0x20; /**< ID for the computer that interconnects ROS and CAN BUS. */

const uint8_t RosCanConstants::VescIds::front_left_vesc = 0x50;  /**< ID for the front left VESC. */
const uint8_t RosCanConstants::VescIds::front_right_vesc = 0x51; /**< ID for the front right VESC. */
const uint8_t RosCanConstants::VescIds::rear_right_vesc = 0x52;  /**< ID for the rear right VESC. */
const uint8_t RosCanConstants::VescIds::rear_left_vesc = 0x53;   /**< ID for the rear left VESC. */

const uint8_t RosCanConstants::VescIds::front_left_stepper = 0x60;  /**< ID for the front left stepper. */
const uint8_t RosCanConstants::VescIds::front_right_stepper = 0x61; /**< ID for the front right stepper. */
const uint8_t RosCanConstants::VescIds::rear_right_stepper = 0x62;  /**< ID for the rear right stepper. */
const uint8_t RosCanConstants::VescIds::rear_left_stepper = 0x63;   /**< ID for the rear left stepper. */

const uint8_t RosCanConstants::VescIds::manipulator_axis_1 = 0x70;  /**< ID for the manipulator base motor. */
const uint8_t RosCanConstants::VescIds::manipulator_axis_2 = 0x71;  /**< ID for the manipulator 2nd axis */
const uint8_t RosCanConstants::VescIds::manipulator_axis_3 = 0x72;  /**< ID for the manipulator 3rd axis */
const uint8_t RosCanConstants::VescIds::manipulator_axis_4 = 0x73;  /**< ID for the manipulator 4th axis */
const uint8_t RosCanConstants::VescIds::manipulator_axis_5 = 0x74;  /**< ID for the manipulator 5th axis */
const uint8_t RosCanConstants::VescIds::manipulator_axis_6 = 0x75;  /**< ID for the manipulator 6th axis */
const uint8_t RosCanConstants::VescIds::manipulator_gripper = 0x76; /**< ID for the manipulator gripper */

const uint8_t RosCanConstants::VescIds::sampler_platform = 0x80;       /**< ID for the sampler platform movement */
const uint8_t RosCanConstants::VescIds::sampler_drill_mov = 0x81;      /**< ID for the sampler drill movement */
const uint8_t RosCanConstants::VescIds::sampler_drill = 0x82;          /**< ID for the sampler drill action */
const uint8_t RosCanConstants::VescIds::sampler_container_a = 0x83;    /**< ID for the sampler container A (position only) */
const uint8_t RosCanConstants::VescIds::sampler_container_b = 0x84;    /**< ID for the sampler container B (position only) */
const uint8_t RosCanConstants::VescIds::sampler_vacuum_suction = 0x85; /**< ID for the sampler vacuum main motor -1:1 duty range */
const uint8_t RosCanConstants::VescIds::sampler_vacuum_a = 0x86;       /**< ID for the sampler vacuum motor A ON-OFF only (duty 0-1) */
const uint8_t RosCanConstants::VescIds::sampler_vacuum_b = 0x87;       /**< ID for the sampler vacuum motor B ON-OFF only (duty 0-1) */

const uint8_t RosCanConstants::VescIds::bms_battery_left = 0x90;   /**< ID for the left battery BMS. */
const uint8_t RosCanConstants::VescIds::bms_battery_center = 0x91; /**< ID for the center battery BMS. */
const uint8_t RosCanConstants::VescIds::bms_battery_right = 0x92;  /**< ID for the right battery BMS. */

const rclcpp::Duration RosCanConstants::max_stm_sync_time = rclcpp::Duration::from_seconds(0.005);
