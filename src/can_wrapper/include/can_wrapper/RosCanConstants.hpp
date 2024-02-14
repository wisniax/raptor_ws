#ifndef ROS_CAN_CONSTANTS_HPP
#define ROS_CAN_CONSTANTS_HPP

#include <string>
#include <ros/ros.h>

/**
 * @brief This struct contains the names of the ROS topics used for communication with the CAN bus.
 */
struct RosCanConstants
{
	struct RosTopics
	{
		static const std::string can_raw_RX;		/**< Topic for all received CAN messages. */
		static const std::string can_raw_TX;		/**< Topic for all sent CAN messages. */
		static const std::string can_set_motor_vel; /**< Topic used for setting the motor velocity. */
		static const std::string can_get_motor_vel; /**< Topic used for getting the real motor velocity. */
		static const std::string can_get_motor_current; /**< Topic used for getting the real motor velocity. */
		static const std::string can_stm_errors;	/**< Topic used for reporting STM32 errors. */
		static const std::string can_stm_init;		/**< Topic used for initializing the STM32. */
	};

	struct VescIds
	{
		static const uint32_t front_left; /**< ID for the front left VESC. */
		static const uint32_t front_right; /**< ID for the front right VESC. */
		static const uint32_t rear_left; /**< ID for the rear left VESC. */
		static const uint32_t rear_right; /**< ID for the rear right VESC. */
	};

	static const ros::Duration max_stm_sync_time; /**< Maximum time to wait for the STM32 to synchronize. */
};
#endif // ROS_CAN_CONSTANTS_HPP
