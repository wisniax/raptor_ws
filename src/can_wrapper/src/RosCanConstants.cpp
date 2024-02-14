#include "can_wrapper/RosCanConstants.hpp"

const std::string RosCanConstants::RosTopics::can_raw_RX = "/CAN/RX/raw";
const std::string RosCanConstants::RosTopics::can_raw_TX = "/CAN/TX/raw";
const std::string RosCanConstants::RosTopics::can_set_motor_vel = "/CAN/TX/set_motor_vel";
const std::string RosCanConstants::RosTopics::can_get_motor_vel = "/CAN/RX/real_motor_vel";
const std::string RosCanConstants::RosTopics::can_get_motor_current = "/CAN/RX/real_motor_curr";
const std::string RosCanConstants::RosTopics::can_stm_errors = "/CAN/RX/stm_errors";
const std::string RosCanConstants::RosTopics::can_stm_init = "/CAN/TX/stm_init";

const uint32_t RosCanConstants::VescIds::front_left = 60;  /**< ID for the front left VESC. */
const uint32_t RosCanConstants::VescIds::front_right = 61; /**< ID for the front right VESC. */
const uint32_t RosCanConstants::VescIds::rear_left = 62;   /**< ID for the rear left VESC. */
const uint32_t RosCanConstants::VescIds::rear_right = 63;  /**< ID for the rear right VESC. */

const ros::Duration RosCanConstants::max_stm_sync_time = ros::Duration(0.005);