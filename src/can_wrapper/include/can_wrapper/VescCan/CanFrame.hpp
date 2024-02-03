#ifndef Vesc_CanFrame_h_
#define Vesc_CanFrame_h_

#include <stdint.h>
#include <memory.h>
#include <assert.h>
#include <type_traits>
#include <boost/endian/conversion.hpp>
#include <boost/endian/arithmetic.hpp>

#include <can_msgs/Frame.h>
#include <CM/CM.h>

// https://github.com/vedderb/bldc/blob/master/documentation/comm_can.md

namespace VescCan
{    
    namespace Consts
    {
        enum class Command : uint8_t
        {
            // Direction ROS --> CAN

            SET_DUTY = 0, // arg unit [% / 100]
            SET_CURRENT = 1, // arg unit [A]
            SET_CURRENT_BRAKE = 2, // arg unit [A]
            SET_RPM = 3, // arg unit [RPM]
            SET_POS = 4, // arg unit [Deg]
            SET_CURRENT_REL = 10, // arg unit [% / 100]
            SET_CURRENT_BRAKE_REL = 11, // arg unit [% / 100]
            SET_CURRENT_HANDBRAKE = 12, // arg unit [A]
            SET_CURRENT_HANDBRAKE_REL = 13, // arg unit [% / 100]

            // Direction ROS <-- CAN

            STATUS_1 = 9,
            STATUS_2 = 14,
            STATUS_3 = 15,
            STATUS_4 = 16,
            STATUS_5 = 27,
            STATUS_6 = 28
        };

        const int32_t SET_DUTY_SCALE = 100000;
        const int32_t SET_CURRENT_SCALE = 1000;
        const int32_t SET_CURRENT_BRAKE_SCALE = 1000;
        const int32_t SET_RPM_SCALE = 1;
        const int32_t SET_POS_SCALE = 1000000;
        const int32_t SET_CURRENT_REL_SCALE = 100000;
        const int32_t SET_CURRENT_BRAKE_REL_SCALE = 100000;
        const int32_t SET_CURRENT_HANDBRAKE_SCALE = 1000;
        const int32_t SET_CURRENT_HANDBRAKE_REL_SCALE = 100000;

        const int32_t STATUS_1_ERPM_SCALE = 1;
        const int16_t STATUS_1_CURRENT_SCALE = 10;
        const int16_t STATUS_1_DUTYCYCLE_SCALE = 1000;

        const int32_t STATUS_2_AMPHOURS_SCALE = 10000;
        const int32_t STATUS_2_AMPHOURSCHG_SCALE = 10000;

        const int32_t STATUS_3_WATTHOURS_SCALE = 10000;
        const int32_t STATUS_3_WATTHOURSCHG_SCALE = 10000;

        const int16_t STATUS_4_TEMPFET_SCALE = 10;
        const int16_t STATUS_4_TEMPMOTOR_SCALE = 10;
        const int16_t STATUS_4_CURRENTIN_SCALE = 10;
        const int16_t STATUS_4_PIDPOS_SCALE = 50;

        const int32_t STATUS_5_TACHOMETER_SCALE = 6;
        const int16_t STATUS_5_VOLTSIN_SCALE = 10;

        const int16_t STATUS_6_ADC1_SCALE = 1000;
        const int16_t STATUS_6_ADC2_SCALE = 1000;
        const int16_t STATUS_6_ADC3_SCALE = 1000;
        const int16_t STATUS_6_PPM_SCALE = 1000;
    }

    struct CanFrame 
    {
    public:

        //ready to send frame
        explicit CanFrame(uint8_t vescID, Consts::Command commandID, boost::endian::big_int32_buf_t commandArg) :
            vescID(vescID), commandID(commandID), commandArg(commandArg)
        {}

        //commandArg is changed from Native Endian to Big Endian
        explicit CanFrame(uint8_t vescID, Consts::Command commandID, int32_t commandArg) :
            vescID(vescID), commandID(commandID), commandArg(boost::endian::native_to_big(commandArg))
        {}

        CanFrame() : 
            CanFrame(0,Consts::Command::SET_DUTY,0)
        {}

        operator CM_CanMessage()
        {
            CM_CanMessage cm;
            BOOST_STATIC_ASSERT(sizeof(CM_CanMessage) == sizeof(CanFrame)); //check if there is slightest chance it will work
            //trust me. I am an engineer! I think we'll put this thing right here!
            memcpy(&cm,this,sizeof(CanFrame));
            return cm;
        }

        operator can_msgs::Frame()
        {
        	can_msgs::Frame canFrame;
            //trust me. I am an engineer! with epic skill and epic gear!
        	canFrame.id = *reinterpret_cast<uint32_t*>(&vescID);
        	canFrame.header.stamp = ros::Time::now();
        	canFrame.dlc = DATA_LEN;
            //trust me. I am an engineer! oh. I think i'm outta here!
        	memcpy(canFrame.data.begin(), &commandArg, CM_CAN_DLEN_MAX);
        	return canFrame;
        }

        // address section
        uint8_t vescID;
        Consts::Command commandID;
        const uint16_t _unused0 = 0;

        // dataLen
        const uint8_t DATA_LEN = 4;

        // data section
        boost::endian::big_int32_buf_t commandArg;
        const uint32_t _unused1 = 0;
    };

    struct CAN_PACKET_STATUS {};
    struct STATUS_1 : private CAN_PACKET_STATUS
    {
        uint32_t eRpm; // unit [RPM]
        uint16_t current; // unit [A]
        uint16_t dutyCucle; // unit [% / 100]
    };

    struct STATUS_2 : private CAN_PACKET_STATUS
    {
        uint32_t ampHours; // unit [Ah]
        uint32_t ampHoursChg; // unit [Ah]
    };

    struct STATUS_3 : private CAN_PACKET_STATUS
    {
        uint32_t wattHours; // unit [Wh]
        uint32_t wattHoursChg; // unit [Wh]
    };
    
    struct STATUS_4 : private CAN_PACKET_STATUS
    {
        uint16_t tempFet; // unit [DegC]
        uint16_t tempMotor; // unit [DegC]
        uint16_t currentIn; // unit [A]
        uint16_t pidPos; // unit [Deg]
    };

    struct STATUS_5 : private CAN_PACKET_STATUS
    {
        uint32_t tachometer; // unit [EREV]
        uint32_t voltsIn; // unit [V]
    };

    struct STATUS_6 : private CAN_PACKET_STATUS
    {
        uint16_t adc1; // unit [V]
        uint16_t adc2; // unit [V]
        uint16_t adc3; // unit [V]
        uint16_t ppm; // unit [% / 100]
    };

    //temp for now
    template <class STATUS>
    STATUS getStatus(CM_CanMessage cm)
    {
        BOOST_STATIC_ASSERT(std::is_base_of<CAN_PACKET_STATUS,STATUS>().value);
        BOOST_ASSERT(cm.dataLen == sizeof(STATUS));
        return *reinterpret_cast<STATUS*>(&cm.data.raw);
    }
}

#endif //Vesc_CanFrame_h_