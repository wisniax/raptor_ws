#include "can_wrapper/RosCanConstants.hpp"

const std::string RosCanConstants::RosTopics::can_raw_RX = "/CAN/RX/raw";
const std::string RosCanConstants::RosTopics::can_raw_TX = "/CAN/TX/raw";
const std::string RosCanConstants::RosTopics::can_set_motor_vel = "/CAN/TX/set_motor_vel";
const std::string RosCanConstants::RosTopics::can_get_motor_vel = "/CAN/RX/real_motor_vel";
const std::string RosCanConstants::RosTopics::can_get_motor_current = "/CAN/RX/real_motor_curr";
const std::string RosCanConstants::RosTopics::can_stm_errors = "/CAN/RX/stm_errors";
const std::string RosCanConstants::RosTopics::can_stm_init = "/CAN/TX/stm_init";
const std::string RosCanConstants::RosTopics::can_vesc_status = "/CAN/RX/vesc_status";
const std::string RosCanConstants::RosTopics::can_manipulator_ctl = "/CAN/TX/manipulator_ctl";
const std::string RosCanConstants::RosTopics::can_probe_status = "/CAN/RX/probe_status";

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

const uint8_t RosCanConstants::VescIds::bms_battery_left = 0x90; /**< ID for the left battery BMS. */
const uint8_t RosCanConstants::VescIds::bms_battery_center = 0x91; /**< ID for the center battery BMS. */
const uint8_t RosCanConstants::VescIds::bms_battery_right = 0x92; /**< ID for the right battery BMS. */

const ros::Duration RosCanConstants::max_stm_sync_time = ros::Duration(0.005);
