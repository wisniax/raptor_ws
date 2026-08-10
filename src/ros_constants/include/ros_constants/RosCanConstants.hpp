#ifndef ROS_CAN_CONSTANTS_HPP
#define ROS_CAN_CONSTANTS_HPP

#include <string>
#include "rclcpp/rclcpp.hpp"

/**
 * @brief This struct contains the names of the ROS topics used for communication with the CAN bus.
 */
struct RosCanConstants
{
	struct RosTopics
	{
		static const std::string can_raw_RX;					/**< Topic for all received CAN messages. */
		static const std::string can_raw_TX;					/**< Topic for all sent CAN messages. */
		static const std::string can_set_motor_vel;				/**< Topic used for setting the motor velocity. */
		static const std::string can_get_motor_vel;				/**< Topic used for getting the real motor velocity. */
		static const std::string can_get_motor_current;			/**< Topic used for getting the real motor velocity. */
		static const std::string can_stm_errors;				/**< Topic used for reporting STM32 errors. */
		static const std::string can_stm_init;					/**< Topic used for initializing the STM32. */
		static const std::string can_vesc_status;				/**< Topic with vesc motor status */
		static const std::string can_manipulator_ctl;			/**< Topic used for manipulator 'manipulation' */
		static const std::string can_sampler_status;			/**< Topic with sampler status */
		static const std::string can_battery_info;				/**< Topic with battery (BMS & HOTSWAP) status */
		static const std::string can_calibration_motor_command; /**< Topic used for VESC commands during calibration */
		static const std::string mqtt_rover_status;				/**< Topic for rover status from MQTT. */
		static const std::string mqtt_sampler_control;			/**< Topic for sampler control from MQTT. */
		static const std::string mqtt_calibrate_axis;			/**< Topic used to calibrate motors (wheels, manipulator) */
	};

	struct VescIds
	{
		static const uint8_t ros_can_host; /**< ID for the computer that interconnects ROS and CAN BUS. */

		// 0x50 - 0x5f - range for wheel vesc motors
		static const uint8_t front_left_vesc;  /**< ID for the front left VESC. */
		static const uint8_t front_right_vesc; /**< ID for the front right VESC. */
		static const uint8_t rear_left_vesc;   /**< ID for the rear left VESC. */
		static const uint8_t rear_right_vesc;  /**< ID for the rear right VESC. */

		// 0x60 - 0x6f - range for wheel stepper motors
		static const uint8_t front_left_stepper;  /**< ID for the front left stepper. */
		static const uint8_t front_right_stepper; /**< ID for the front right stepper. */
		static const uint8_t rear_left_stepper;	  /**< ID for the rear left stepper. */
		static const uint8_t rear_right_stepper;  /**< ID for the rear right stepper. */

		// 0x70 - 0x7f - range for manipulator motors
		static const uint8_t manipulator_axis_1;  /**< ID for the manipulator base motor. */
		static const uint8_t manipulator_axis_2;  /**< ID for the manipulator 2nd axis */
		static const uint8_t manipulator_axis_3;  /**< ID for the manipulator 3rd axis */
		static const uint8_t manipulator_axis_4;  /**< ID for the manipulator 4th axis */
		static const uint8_t manipulator_axis_5;  /**< ID for the manipulator 5th axis */
		static const uint8_t manipulator_axis_6;  /**< ID for the manipulator 6th axis */
		static const uint8_t manipulator_gripper; /**< ID for the manipulator gripper */

		// 0x80 - 0x8f - range for the sampler
		static const uint8_t sampler_platform;		 /**< ID for the sampler platform movement */
		static const uint8_t sampler_drill_mov;		 /**< ID for the sampler drill movement */
		static const uint8_t sampler_drill;			 /**< ID for the sampler drill action */
		static const uint8_t sampler_container_a;	 /**< ID for the sampler container A (position only) */
		static const uint8_t sampler_container_b;	 /**< ID for the sampler container B (position only) */
		static const uint8_t sampler_vacuum_suction; /**< ID for the sampler vacuum main motor -1:1 duty range */
		static const uint8_t sampler_vacuum_a;		 /**< ID for the sampler vacuum motor A ON-OFF only (duty 0-1) */
		static const uint8_t sampler_vacuum_b;		 /**< ID for the sampler vacuum motor B ON-OFF only (duty 0-1) */

		// 0x90 - 0x9f - range for the bms
		static const uint8_t bms_battery_left;	 /**< ID for the left battery BMS. */
		static const uint8_t bms_battery_center; /**< ID for the center battery BMS. */
		static const uint8_t bms_battery_right;	 /**< ID for the right battery BMS. */
	};

	static const rclcpp::Duration max_stm_sync_time; /**< Maximum time to wait for the STM32 to synchronize. */
};
#endif // ROS_CAN_CONSTANTS_HPP
