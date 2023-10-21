//Last modified by: Marcin Walczyk/12.10.2023 

#ifndef CANMESSAGE_H
#define CANMESSAGE_H

#include <linux/can.h>
#include <stdint.h>

struct CanMessage
{
	canid_t address;
	uint8_t dataLength;
	union __data
	{
		uint8_t raw[CAN_MAX_DLEN];
		
		struct __callbackFormat
		{
    		uint16_t messageID : 12;
    		int8_t errorCode : 4;
		} callback;
		
	} data __attribute__((aligned(8)));
};

#endif //CANMESSAGE_H