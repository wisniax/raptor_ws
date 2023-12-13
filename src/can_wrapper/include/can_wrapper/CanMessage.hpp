#ifndef CANMESSAGE_H
#define CANMESSAGE_H

#include <linux/can.h>
#include <stdint.h>
#include <can_msgs/Frame.h>
#include <ros/time.h>
#include <string>

/**
 * @brief CAN Frame with human readable data format
 */
struct CanMessage
{
	/**
	 * @brief Masks for CanMessage
	 */
	enum Masks : canid_t
	{
		All_Nodes = 0x60F,
		Adress_Families = 0x7F0,
	};

	/**
	 * @brief CAN Address for CanMessage
	 */
	enum Address : canid_t
	{
		Stm_Left = 0x5,
		Stm_Right = 0x6,
		Stm_Arm_Axis_123 = 0x7,
		Stm_Arm_Axis_456 = 0x8,

		// Adress_Families

		Error = 0x1 << 4,					  // RX ERRORS (ROS <-- CAN)
		Init = 0x2 << 4,					  // TX Init (ROS --> CAN)
		Status = 0x3 << 4,					  // RX (ROS <-- CAN)
		Motor_Control = 0x4 << 4,			  // TX (ROS --> CAN)
		Encoder_Velocity_Feedback = 0x5 << 4, // RX (ROS <-- CAN)
		Encoder_Distance_Traveled = 0x6 << 4, // RX (ROS <-- CAN)

		Invalid = 0x7FF,
	};

	struct mode_t
	{
		uint8_t cont_mode : 2;
		uint8_t __unused0 : 1;
		uint8_t reason : 1;
	};

	/**
	 * @brief Message Format for initialization and setup.
	 * For usage see other Message Formats with init settings (containing stm_init_type_id enum) eg. @ref set_motor_vel_t .
	 */
	struct stm_init_t
	{
		uint8_t type_id : 8;
		float var;
	} __attribute__((packed));;

	/**
	 * @brief Struct representing error codes for various components of the motor controller.
	 */
	struct node_errors_t
	{
		uint8_t unique_err : 4;
		uint8_t select_err : 4;
		uint8_t rpm_scale_err : 4;
		uint8_t motor_a_reg_err : 4;
		uint8_t motor_b_reg_err : 4;
		uint8_t motor_c_reg_err : 4;
	};

	/**
	 * @brief Message Format format for commanding motors.
	 * motor_*_vel meaning is dependant on init settings (Target RPM or Target PWM)
	 * Initializable (see @ref stm_init_type_id)
	 */
	struct set_motor_vel_t
	{
		/**
		 * Possible mode::count_mode values
		 */
		enum mode_cont_mode : uint8_t
		{
			/**
			 * motor_*_vel is wrap unsigned value (0-2047) into scale (0 - max_rpm) used to set RPM Target for motor.
			 * ( max_rpm = (2^11-1) / rpm_scale )
			 */
			TargetModeRpm = 0b00,
			/**
			 * (dunno, wip documentation TODO CORRECT)
			 */
			PreciseMode = 0b01,
			/**
			 * unused.
			 */
			__unused0 = 0b10,
			/**
			 * @brief motor_*_vel is used to set PWM Target for motor.
			 */
			TargetModePwm = 0b11
		};

		/**
		 * Possible stm_init::type_id values with stm_init::frame_a/b params usage
		 */
		enum stm_init_type_id : uint8_t
		{
			/**
			 * @brief Setup frequency for Update and Upkeep.
			 * Parameters in @ref stm_init_t::frame_a
			 * @param x update frequency (x * 1/100 Hz)
			 * @param y upkeep frequency (y * 1/100 s)
			 */
			Freq = 0x10,
			/**
			 * @brief Setup rpm_scale.
			 * When sending RPM target, this is used to wrap unsigned value (0-2047) into scale (0 - max_rpm)
			 * ( max_rpm = (2^11-1) / rpm_scale )
			 * Parameters in @ref stm_init_t::frame_b
			 * @param var rpm_scale value
			 */
			RpmScale = 0x11,
			/**
			 * @brief Regulator A param (dunno, wip documentation TODO CORRECT).
			 * Parameters in @ref stm_init_t::frame_b
			 * @param var param value
			 */
			MotorA_Regulator = 0x20,
			/**
			 * @brief Regulator B param (dunno, wip documentation TODO CORRECT).
			 * Parameters in @ref stm_init_t::frame_b
			 * @param var param value
			 */
			MotorB_Regulator = 0x30,
			/**
			 * @brief Regulator C param (dunno, wip documentation TODO CORRECT).
			 * Parameters in @ref stm_init_t::frame_b
			 * @param var param value
			 */
			MotorC_Regulator = 0x40
		};

		/**
		 * @brief use @c get_mode() method or see @c CanMessage.data.mode .
		 */
		uint8_t __mode : 4;

		/**
		 * @brief Get the @c mode_t struct
		 * @return mode_t*
		 */
		mode_t *get_mode() { return (mode_t *)this; }

		uint16_t motor_A_vel : 11;
		bool motor_A_dir : 1;

		uint16_t motor_B_vel : 11;
		bool motor_B_dir : 1;

		uint16_t motor_C_vel : 11;
		bool motor_C_dir : 1;
	} __attribute__((packed));;

	/**
	 * @brief Message Format format for motors feedback.
	 * motor_*_vel meaning is dependant on init settings (Target RPM or Target RPM (new version))
	 * Initializable (see @ref stm_init_type_id)
	 */
	struct get_motor_vel_t
	{
		/**
		 * Possible mode::count_mode values
		 */
		enum mode_cont_mode : uint8_t
		{
			/**
			 * @brief motor_*_vel is RPM Feed from motor.
			 * wraped unsigned value (0-32767) into scale (0 - max_rpm)
			 * where max_rpm = (2^11-1) / rpm_scale .
			 */
			FeedModeRpm = 0b00,
			/**
			 * @brief motor_*_vel is RPM Feed (new version) from motor.
			 * wraped unsigned value (0-32767) into scale (0 - max_rpm)
			 * where max_rpm = (dunno, bad documentation TODO CORRECT)
			 */
			FeedModeRpmNew = 0b01,
			/**
			 * unused.
			 */
			__unused0 = 0b10,
			/**
			 * unused.
			 */
			__unused1 = 0b11
		};

		/**
		 * Possible stm_init::type_id values with stm_init::frame_a/b params usage
		 */
		enum stm_init_type_id : uint8_t
		{
			/**
			 * @brief Setup frequency for Update and Upkeep.
			 * Parameters in @ref stm_init_t::frame_a
			 * @param x update frequency (x * 1/100 Hz)
			 * @param y upkeep frequency (y * 1/100 s)
			 */
			Freq = 0x10,
			/**
			 * @brief Setup rpm_scale.
			 * When reciving RPM feed, this is used to wrap unsigned value (0-32767) into scale (0 - max_rpm)
			 * max_rpm is dependant on FeedMode. see @ref mode_cont_mode .
			 * Parameters in @ref stm_init_t::frame_b
			 * @param var rpm_scale value
			 */
			RpmScale = 0x11
		};

		/**
		 * @brief use @c get_mode() method or see @c CanMessage.data.mode .
		 */
		uint16_t __mode : 4;

		/**
		 * @brief Get the @c mode_t struct
		 * @return mode_t*
		 */
		mode_t *get_mode() { return (mode_t *)this; }

		uint16_t err_code : 4;

		uint16_t motor_A_vel : 15;
		uint16_t motor_A_dir : 1;

		uint16_t motor_B_vel : 15;
		uint8_t motor_B_dir : 1;

		uint16_t motor_C_vel : 15;
		uint8_t motor_C_dir : 1;
	} __attribute__((packed));;

	/**
	 * Message Format for checkup/reading data from peripherals
	 * (dunno, wip documentation TODO CORRECT)
	 */
	struct periph_t
	{
		/**
		 * @brief use @c get_mode() method or see @c CanMessage.data.mode .
		 */
		uint8_t __mode : 4;

		/**
		 * @brief Get the @c mode_t struct
		 * @return mode_t*
		 */
		mode_t *get_mode() { return (mode_t *)this; }

		uint16_t periph : 12;
	};

	/**
	 * @brief Zero-init constructor
	 */
	CanMessage()
	{
		address = Address::Invalid;
		dataLen = 0;
		memset(&data, 0, CAN_MAX_DLEN);
	}

	/**
	 * @brief ROS CAN Frame based constructor
	 */
	CanMessage(const can_msgs::Frame *frame)
	{
		address = (Address)frame->id;
		dataLen = frame->dlc;
		memcpy(&data, frame->data.data(), CAN_MAX_DLEN);
	}

	explicit operator can_msgs::Frame() const
	{
		can_msgs::Frame ret;
		ret.id = (uint32_t)address;
		ret.header.stamp = ros::Time::now();
		ret.dlc = dataLen;
		memcpy(ret.data.begin(), &data, CAN_MAX_DLEN);
		return ret;
	}

	canid_t address;
	uint8_t dataLen;
	union data_t
	{
		/**
		 * @brief raw data.
		 */
		uint8_t raw[CAN_MAX_DLEN];

		mode_t mode;

		stm_init_t stm_init;

		node_errors_t node_errors;

		set_motor_vel_t set_motor_vel;

		get_motor_vel_t get_motor_vel;

		periph_t periph;

	} data __attribute__((aligned(8)));
};

#endif // CANMESSAGE_H