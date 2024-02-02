#include <stdint.h>
#include <memory.h>
#include <assert.h>

#include <can_msgs/Frame.h>
#include <CM/CM.h>

// https://github.com/vedderb/bldc/blob/master/documentation/comm_can.md

namespace VescCan
{
    enum class VescCommand : uint8_t
    {
        CAN_PACKET_SET_DUTY = 0,
        CAN_PACKET_SET_CURRENT = 1,
        CAN_PACKET_SET_CURRENT_BRAKE = 2,
        CAN_PACKET_SET_RPM = 3,
        CAN_PACKET_SET_POS = 4,
        CAN_PACKET_SET_CURRENT_REL = 10,
        CAN_PACKET_SET_CURRENT_BRAKE_REL = 11,
        CAN_PACKET_SET_CURRENT_HANDBRAKE = 12,
        CAN_PACKET_SET_CURRENT_HANDBRAKE_REL = 13
    };

    const int32_t CAN_PACKET_SET_DUTY_SCALE = 100000;
    const int32_t CAN_PACKET_SET_CURRENT_SCALE = 1000;
    const int32_t CAN_PACKET_SET_CURRENT_BRAKE_SCALE = 1000;
    const int32_t CAN_PACKET_SET_RPM_SCALE = 1;
    const int32_t CAN_PACKET_SET_POS_SCALE = 1000000;
    const int32_t CAN_PACKET_SET_CURRENT_REL_SCALE = 100000;
    const int32_t CAN_PACKET_SET_CURRENT_BRAKE_REL_SCALE = 100000;
    const int32_t CAN_PACKET_SET_CURRENT_HANDBRAKE_SCALE = 1000;
    const int32_t CAN_PACKET_SET_CURRENT_HANDBRAKE_REL_SCALE = 100000;

    struct CanFrame 
    {
    public:

        CanFrame(uint8_t vescID, VescCommand commandID, uint32_t commandArg) :
            vescID(vescID), commandID(commandID), commandArg(commandArg)
        {}
        CanFrame() : 
            CanFrame(0,VescCommand::CAN_PACKET_SET_POS,0)
        {}

        operator CM_CanMessage()
        {
            CM_CanMessage cm;
            assert(sizeof(CM_CanMessage) == sizeof(CanFrame));
            memcpy(&cm,this,sizeof(CanFrame));
            return cm;
        }

        operator can_msgs::Frame()
        {
        	can_msgs::Frame canFrame;
            //trust me. I am an engineer! I think we'll put this thing right here.
        	canFrame.id = *reinterpret_cast<uint32_t*>(&vescID);
        	canFrame.header.stamp = ros::Time::now();
        	canFrame.dlc = DATA_LEN;
            //trust me. I am an engineer! with epic skill and epic gear!
        	memcpy(canFrame.data.begin(), &commandArg, CM_CAN_DLEN_MAX);
        	return canFrame;
        }

        // address section
        uint8_t vescID;
        VescCommand commandID;
        const uint16_t _unused0 = 0;

        // dataLen
        const uint8_t DATA_LEN = 4;

        // data section
        uint32_t commandArg;
        const uint32_t _unused1 = 0;
    } __attribute__ ((packed));;
}