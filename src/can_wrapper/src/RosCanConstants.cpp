#include "can_wrapper/RosCanConstants.hpp"

const std::string RosCanConstants::RosTopics::can_raw_RX = "/CAN/RX/raw";
const std::string RosCanConstants::RosTopics::can_raw_TX = "/CAN/TX/raw";
const std::string RosCanConstants::RosTopics::can_set_motor_vel = "/CAN/TX/set_motor_vel";
const std::string RosCanConstants::RosTopics::can_get_motor_vel = "/CAN/RX/real_motor_vel";
const std::string RosCanConstants::RosTopics::can_get_motor_current = "/CAN/RX/real_motor_curr";
const std::string RosCanConstants::RosTopics::can_stm_errors = "/CAN/RX/stm_errors";
const std::string RosCanConstants::RosTopics::can_stm_init = "/CAN/TX/stm_init";

const ros::Duration RosCanConstants::max_stm_sync_time = ros::Duration(0.005);