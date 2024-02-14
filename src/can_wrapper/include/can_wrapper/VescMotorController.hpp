#ifndef VescMotorController_h_
#define VescMotorController_h_

#include <ros/ros.h>
#include <unordered_map>
#include <boost/algorithm/algorithm.hpp>
#include <boost/shared_ptr.hpp>
#include <can_wrapper/VescStatus.h>
#include <can_wrapper/RosCanConstants.hpp>
#include <can_wrapper/VescCan/CanFrame.hpp>
#include <can_wrapper/VescCan/CanFrameFactory.hpp>
#include <can_wrapper/VescCan/Converter.hpp>

struct MotorStatusKey
    {
        uint8_t vescId;
        VescCan::Consts::Command commandId;
        MotorStatusKey() = default;
        MotorStatusKey(uint8_t vescId, VescCan::Consts::Command commandId) :
            vescId(vescId), commandId(commandId)
        {}

        bool operator==(const MotorStatusKey &rhs) const
        {
            return vescId == rhs.vescId && commandId == rhs.commandId;
        }
    };

    struct MotorStatusValue
    {
        VescCan::CanFrame vescFrame;
        ros::Time recivedTime;
        MotorStatusValue() = default;
        MotorStatusValue(VescCan::CanFrame vescFrame, ros::Time recivedTime) :
            vescFrame(vescFrame), recivedTime(recivedTime)
        {}
    };

template<class T>
struct MyHash;

template<>
struct MyHash<MotorStatusKey>
{
public:
    std::size_t operator()(MotorStatusKey const& key) const 
    {
        std::size_t h1 = std::hash<uint8_t>()(key.vescId);
        std::size_t h2 = std::hash<uint8_t>()(uint8_t(key.commandId));
        return h1 ^ (h2 << 1);
    }
};

class VescMotorController
{
public:
    VescMotorController()
    {
        mStatusGrabber = mNodeHandle.subscribe<can_msgs::Frame>(RosCanConstants::RosTopics::can_raw_RX,1024,&VescMotorController::statusGrabber,this);
        mStatusPublisher = mNodeHandle.advertise<can_wrapper::VescStatus>(RosCanConstants::RosTopics::can_vesc_status,256);
    }

    void statusGrabber(const can_msgs::Frame::ConstPtr &frame)
    {
        if(!VescCan::CanFrame::isValidVescCanFrame(frame->id))
            return;

        auto vescFrame = VescCan::CanFrame(frame);
        auto key = MotorStatusKey(vescFrame.vescID, vescFrame.commandID);
        auto value = MotorStatusValue(vescFrame,frame->header.stamp);

        auto findResult = mMotorStatus.find(key);

        if( findResult == mMotorStatus.cend() )
        {
            mMotorStatus.insert(std::pair<MotorStatusKey,MotorStatusValue>(key,value));
        }
        else
        {
            if(key.commandId == VescCan::Consts::Command::STATUS_1)
                sendUpdate(key.vescId);
            mMotorStatus[key] = value;
        }
    }

    void sendUpdate(uint8_t vescId)
    {
        MotorStatusKey key;

        can_wrapper::VescStatus status;

        key = MotorStatusKey(vescId, VescCan::Consts::Command::STATUS_1);
        if(mMotorStatus.find(key) != mMotorStatus.cend())
        {
            status.ERPM = VescCan::ConverterI32<VescCan::Consts::STATUS_1_ERPM_SCALE>::decode(mMotorStatus[key].vescFrame.data.status1.eRpm);
            status.Current = VescCan::ConverterF16<VescCan::Consts::STATUS_1_CURRENT_SCALE>::decode(mMotorStatus[key].vescFrame.data.status1.current);
            status.DutyCycle = VescCan::ConverterF16<VescCan::Consts::STATUS_1_DUTYCYCLE_SCALE>::decode(mMotorStatus[key].vescFrame.data.status1.dutyCucle);
        }

        key = MotorStatusKey(vescId, VescCan::Consts::Command::STATUS_2);
        if(mMotorStatus.find(key) != mMotorStatus.cend())
        {
            status.AhUsed = VescCan::ConverterF32<VescCan::Consts::STATUS_2_AMPHOURS_SCALE>::decode(mMotorStatus[key].vescFrame.data.status2.ampHours);
            status.AhCharged = VescCan::ConverterF32<VescCan::Consts::STATUS_2_AMPHOURSCHG_SCALE>::decode(mMotorStatus[key].vescFrame.data.status2.ampHoursChg);
        }

        key = MotorStatusKey(vescId, VescCan::Consts::Command::STATUS_3);
        if(mMotorStatus.find(key) != mMotorStatus.cend())
        {
            status.WhUsed = VescCan::ConverterF32<VescCan::Consts::STATUS_3_WATTHOURS_SCALE>::decode(mMotorStatus[key].vescFrame.data.status3.wattHours);
            status.WhCharged = VescCan::ConverterF32<VescCan::Consts::STATUS_3_WATTHOURSCHG_SCALE>::decode(mMotorStatus[key].vescFrame.data.status3.wattHoursChg);
        }

        key = MotorStatusKey(vescId, VescCan::Consts::Command::STATUS_4);
        if(mMotorStatus.find(key) != mMotorStatus.cend())
        {
            status.TempFet = VescCan::ConverterF16<VescCan::Consts::STATUS_4_TEMPFET_SCALE>::decode(mMotorStatus[key].vescFrame.data.status4.tempFet);
            status.TempMotor = VescCan::ConverterF16<VescCan::Consts::STATUS_4_TEMPMOTOR_SCALE>::decode(mMotorStatus[key].vescFrame.data.status4.tempMotor);
            status.CurrentIn = VescCan::ConverterF16<VescCan::Consts::STATUS_4_CURRENTIN_SCALE>::decode(mMotorStatus[key].vescFrame.data.status4.currentIn);
            status.PidPos = VescCan::ConverterF16<VescCan::Consts::STATUS_4_PIDPOS_SCALE>::decode(mMotorStatus[key].vescFrame.data.status4.pidPos);
        }

        key = MotorStatusKey(vescId, VescCan::Consts::Command::STATUS_5);
        if(mMotorStatus.find(key) != mMotorStatus.cend())
        {
            status.Tachometer = VescCan::ConverterF32<VescCan::Consts::STATUS_5_TACHOMETER_SCALE>::decode(mMotorStatus[key].vescFrame.data.status5.tachometer);
            status.VoltsIn = VescCan::ConverterF16<VescCan::Consts::STATUS_5_VOLTSIN_SCALE>::decode(mMotorStatus[key].vescFrame.data.status5.voltsIn);
        }

        ROS_INFO("Published status!");
        lastSendTime = ros::Time::now();
        mStatusPublisher.publish(status);
    }

    void clear()
    {
        mMotorStatus.clear();
    }

private:

    ros::Time lastSendTime;

    std::unordered_map<MotorStatusKey, MotorStatusValue, MyHash<MotorStatusKey>> mMotorStatus;

    ros::NodeHandle mNodeHandle;

    ros::Subscriber mStatusGrabber;
    ros::Publisher mStatusPublisher;

    ros::Timer mMotorCommandTimer;
    
};



#endif //VescMotorController_h_