#include "can_wrapper/RosCanConstants.hpp"

const std::string RosCanConstants::RosTopics::can_raw_RX = "/CAN/RX/raw";
const std::string RosCanConstants::RosTopics::can_raw_TX = "/CAN/TX/raw";
const std::string RosCanConstants::RosTopics::can_set_motor_vel = "/CAN/TX/set_motor_vel";
const std::string RosCanConstants::RosTopics::can_get_motor_vel = "/CAN/RX/real_motor_vel";
const std::string RosCanConstants::RosTopics::can_get_motor_current = "/CAN/RX/real_motor_curr";
const std::string RosCanConstants::RosTopics::can_stm_errors = "/CAN/RX/stm_errors";
const std::string RosCanConstants::RosTopics::can_stm_init = "/CAN/TX/stm_init";
const std::string RosCanConstants::RosTopics::can_vesc_status = "/CAN/RX/vesc_status";

const uint8_t RosCanConstants::VescIds::front_left_vesc = 0x50;  /**< ID for the front left VESC. */
const uint8_t RosCanConstants::VescIds::front_right_vesc = 0x51; /**< ID for the front right VESC. */
const uint8_t RosCanConstants::VescIds::rear_right_vesc = 0x52;  /**< ID for the rear right VESC. */
const uint8_t RosCanConstants::VescIds::rear_left_vesc = 0x53;   /**< ID for the rear left VESC. */

const uint8_t RosCanConstants::VescIds::front_left_stepper = 0x60;  /**< ID for the front left stepper. */
const uint8_t RosCanConstants::VescIds::front_right_stepper = 0x61; /**< ID for the front right stepper. */
const uint8_t RosCanConstants::VescIds::rear_right_stepper = 0x62;  /**< ID for the rear right stepper. */
const uint8_t RosCanConstants::VescIds::rear_left_stepper = 0x63;   /**< ID for the rear left stepper. */

const ros::Duration RosCanConstants::max_stm_sync_time = ros::Duration(0.005);
