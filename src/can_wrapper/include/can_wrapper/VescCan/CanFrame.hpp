#ifndef VescCan_CanFrame_h_
#define VescCan_CanFrame_h_

#include <stdint.h>
#include <memory.h>
#include <assert.h>
#include <linux/can.h>
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

        constexpr int32_t SET_DUTY_SCALE = 100000;
        constexpr int32_t SET_CURRENT_SCALE = 1000;
        constexpr int32_t SET_CURRENT_BRAKE_SCALE = 1000;
        constexpr int32_t SET_RPM_SCALE = 1;
        constexpr int32_t SET_POS_SCALE = 1000000;
        constexpr int32_t SET_CURRENT_REL_SCALE = 100000;
        constexpr int32_t SET_CURRENT_BRAKE_REL_SCALE = 100000;
        constexpr int32_t SET_CURRENT_HANDBRAKE_SCALE = 1000;
        constexpr int32_t SET_CURRENT_HANDBRAKE_REL_SCALE = 100000;

        constexpr int32_t STATUS_1_ERPM_SCALE = 1;
        constexpr int16_t STATUS_1_CURRENT_SCALE = 10;
        constexpr int16_t STATUS_1_DUTYCYCLE_SCALE = 1000;

        constexpr int32_t STATUS_2_AMPHOURS_SCALE = 10000;
        constexpr int32_t STATUS_2_AMPHOURSCHG_SCALE = 10000;

        constexpr int32_t STATUS_3_WATTHOURS_SCALE = 10000;
        constexpr int32_t STATUS_3_WATTHOURSCHG_SCALE = 10000;

        constexpr int16_t STATUS_4_TEMPFET_SCALE = 10;
        constexpr int16_t STATUS_4_TEMPMOTOR_SCALE = 10;
        constexpr int16_t STATUS_4_CURRENTIN_SCALE = 10;
        constexpr int16_t STATUS_4_PIDPOS_SCALE = 50;

        constexpr int32_t STATUS_5_TACHOMETER_SCALE = 6;
        constexpr int16_t STATUS_5_VOLTSIN_SCALE = 10;

        constexpr int16_t STATUS_6_ADC1_SCALE = 1000;
        constexpr int16_t STATUS_6_ADC2_SCALE = 1000;
        constexpr int16_t STATUS_6_ADC3_SCALE = 1000;
        constexpr int16_t STATUS_6_PPM_SCALE = 1000;
    }

    namespace Data
    {
        struct Status1  
        {
            boost::endian::big_int32_buf_t  eRpm; // unit [RPM]
            boost::endian::big_int16_buf_t  current; // unit [A]
            boost::endian::big_int16_buf_t  dutyCucle; // unit [% / 100]
        };

        struct Status2  
        {
            boost::endian::big_int32_buf_t  ampHours; // unit [Ah]
            boost::endian::big_int32_buf_t  ampHoursChg; // unit [Ah]
        };

        struct Status3  
        {
            boost::endian::big_int32_buf_t  wattHours; // unit [Wh]
            boost::endian::big_int32_buf_t  wattHoursChg; // unit [Wh]
        };

        struct Status4  
        {
            boost::endian::big_int16_buf_t  tempFet; // unit [DegC]
            boost::endian::big_int16_buf_t  tempMotor; // unit [DegC]
            boost::endian::big_int16_buf_t  currentIn; // unit [A]
            boost::endian::big_int16_buf_t  pidPos; // unit [Deg]
        };

        struct Status5  
        {
            boost::endian::big_int32_buf_t  tachometer; // unit [EREV]
            boost::endian::big_int32_buf_t  voltsIn; // unit [V]
        };

        struct Status6  
        {
            boost::endian::big_int16_buf_t  adc1; // unit [V]
            boost::endian::big_int16_buf_t  adc2; // unit [V]
            boost::endian::big_int16_buf_t  adc3; // unit [V]
            boost::endian::big_int16_buf_t  ppm; // unit [% / 100]
        };
    }

    struct CanFrame 
    {
        /// @brief CanFrame with argument as big endian int32 (not scaled). If you feel overwhelmed, use CanFrameFactory 
        /// @param vescID Id of target motor
        /// @param commandID command to send
        /// @param commandArg argument to command
        explicit CanFrame(uint8_t vescID, Consts::Command commandID, const boost::endian::big_int32_buf_t &commandArg) :
            vescID(vescID),
            commandID(commandID),
            dataLen(4)
        { data.commandArg = commandArg; }

        /// @brief CanFrame from CM_CanMessage. Use to read status from motor.
        explicit CanFrame(const CM_CanMessage &cm) :
            vescID(cm.address & 0xFF),
            commandID(static_cast<VescCan::Consts::Command>((cm.address >> 8) & 0xFF)),
            dataLen(cm.dataLen)
        { memcpy(&data.commandArg, cm.data.raw, cm.dataLen); }

        /// @brief CanFrame from can_msgs::Frame. Use to read status from motor.
        /// @param cfp Shared pointer to const can_msgs::Frame. Commonly received from subscriber
        explicit CanFrame(const can_msgs::Frame::ConstPtr &cfp) :
            vescID(cfp->id & 0xFF),
            commandID(static_cast<VescCan::Consts::Command>((cfp->id >> 8) & 0xFF)),
            dataLen(cfp->dlc)
        { memcpy(&data.commandArg, &cfp->data, cfp->dlc); }

        /// @brief CanFrame from can_msgs::Frame. Use to read status from motor.
        explicit CanFrame(const can_msgs::Frame &cf) :
            vescID(cf.id & 0xFF),
            commandID(static_cast<VescCan::Consts::Command>((cf.id >> 8) & 0xFF)),
            dataLen(cf.dlc)
        { memcpy(&data.commandArg, &cf.data, cf.dlc); }
        
        /// @brief Empty one. DIY i guess;
        CanFrame() : 
           vescID(0), commandID(VescCan::Consts::Command::SET_DUTY)
        { memset(&data,0,sizeof(data)); }

        operator CM_CanMessage()
        {
            CM_CanMessage cm;
            memset(&cm,0,sizeof(cm));
            cm.address = vescID | (static_cast<uint32_t>(commandID) << 8) | CAN_EFF_FLAG; //last one is bit flag for extended can frame address
            cm.dataLen = dataLen;
            memcpy(cm.data.raw, &data, dataLen);
            return cm;
        }

        operator can_msgs::Frame()
        {
        	can_msgs::Frame canFrame;
            canFrame.header.stamp = ros::Time::now();
        	canFrame.id = vescID | (static_cast<uint32_t>(commandID) << 8) | CAN_EFF_FLAG; //last one is bit flag for extended can frame address
        	canFrame.dlc = dataLen;
        	memcpy(canFrame.data.begin(), &data, dataLen);
        	return canFrame;
        }

        /// @brief Check if address is valid valid command or status for VescCan::CanFrame
        /// @param canAddress id from can_msgs::Frame or address from CM_CanMessage
        /// @return true for valid one
        static bool isValidVescCanFrame(uint32_t canAddress) noexcept
        {
            //those bits should be zero in valid frame
            if((canAddress & 0x1FFF0000))
                return false;
            
            //commandId should be one of defined commands in valid frame
            uint8_t commandId = (canAddress >> 8) & 0xFF; 
            switch(static_cast<VescCan::Consts::Command>(commandId))
            {
                case VescCan::Consts::Command::SET_DUTY: BOOST_FALLTHROUGH;
                case VescCan::Consts::Command::SET_CURRENT: BOOST_FALLTHROUGH;
                case VescCan::Consts::Command::SET_CURRENT_BRAKE: BOOST_FALLTHROUGH;
                case VescCan::Consts::Command::SET_RPM: BOOST_FALLTHROUGH;
                case VescCan::Consts::Command::SET_POS: BOOST_FALLTHROUGH;
                case VescCan::Consts::Command::SET_CURRENT_REL: BOOST_FALLTHROUGH;
                case VescCan::Consts::Command::SET_CURRENT_BRAKE_REL: BOOST_FALLTHROUGH;
                case VescCan::Consts::Command::SET_CURRENT_HANDBRAKE: BOOST_FALLTHROUGH;
                case VescCan::Consts::Command::SET_CURRENT_HANDBRAKE_REL: BOOST_FALLTHROUGH;
                case VescCan::Consts::Command::STATUS_1: BOOST_FALLTHROUGH;
                case VescCan::Consts::Command::STATUS_2: BOOST_FALLTHROUGH;
                case VescCan::Consts::Command::STATUS_3: BOOST_FALLTHROUGH;
                case VescCan::Consts::Command::STATUS_4: BOOST_FALLTHROUGH;
                case VescCan::Consts::Command::STATUS_5: BOOST_FALLTHROUGH;
                case VescCan::Consts::Command::STATUS_6: 
                    return true;
                default:
                    return false;
            }
        }

        //is this VescCan::CanFrame valid command or status
        bool isValidVescCanFrame() const
        {
            return isValidVescCanFrame(static_cast<uint32_t>(commandID) << 8);
        }


        // address section
        uint8_t vescID;
        Consts::Command commandID;
        const uint16_t _unused0 = 0;

        // dataLen
        uint8_t dataLen;

        // data section
        union CanFrameData_t
        {
            boost::endian::big_int32_buf_t commandArg;
            VescCan::Data::Status1 status1;
            VescCan::Data::Status2 status2;
            VescCan::Data::Status3 status3;
            VescCan::Data::Status4 status4;
            VescCan::Data::Status5 status5;
            VescCan::Data::Status6 status6;
        } data;
    };
}

#endif //VescCan_CanFrame_h_