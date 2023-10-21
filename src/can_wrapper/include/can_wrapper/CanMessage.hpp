//Last modified by: Marcin Walczyk/12.10.2023 

#ifndef CANMESSAGE_H
#define CANMESSAGE_H

#include <linux/can.h>
#include <stdint.h>

namespace cm
{

	enum class Address : canid_t
	{
		Invalid = 0x7FF,
		TX_DriversLeft = 0x10,
		TX_DriversRight = 0x11,
		TX_ArmAxis123 = 0x12,
		TX_ArmAxis456 = 0x13,
		RX_DriversLeft = 0x20,
		RX_DriversRight = 0x21,
		RX_ArmAxis123 = 0x22,
		RX_ArmAxis456 = 0x23
	};

	struct mode_t
	{
		uint8_t cont_mode : 2;
		uint8_t __unused : 1;
		uint8_t reason : 1;
	};

	struct CanMessage
	{
		canid_t address;
		uint8_t dataLength;
		union data_t
		{
			uint8_t raw[CAN_MAX_DLEN];

			struct set_motor_vel_t
			{
				uint8_t mode_cont_mode : 2;
				uint8_t mode_unused0 : 1;
				uint8_t mode_reason : 1;

				uint16_t motor_A_vel : 11;
				uint8_t motor_A_dir : 1;

				uint16_t motor_B_vel : 11;
				uint8_t motor_B_dir : 1;

				uint16_t motor_C_vel : 11;
				uint8_t motor_C_dir : 1;

			} set_motor_vel;

		} data __attribute__((aligned(8)));
	};

};
#endif //CANMESSAGE_H