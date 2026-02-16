#ifndef CALIBRATE_AXIS_H
#define CALIBRATE_AXIS_H

#include "rclcpp/rclcpp.hpp"
#include "rex_interfaces/msg/vesc_status.hpp"
#include "rex_interfaces/msg/rover_status.hpp"
#include "rex_interfaces/msg/battery_info.hpp"
#include "rex_interfaces/msg/calibrate_axis.hpp"
#include "rex_interfaces/msg/vesc_motor_command.hpp"
#include "ros_constants/RosCanConstants.hpp"
#include <cmath>
#include <limits>
#include <array>
#include <map>
#include <vector>
#include <algorithm>
#include <functional>

extern "C"
{
#include <libVescCan/VESC.h>
}

struct MotorStatusStamped
{
    float position;
    int erpm;
    rclcpp::Time receivedAt;
};

enum class Mode
{
    SetVelocity,
    SetPos,
    Hold,
    Nothing
};

class CalibrateAxis : public rclcpp::Node
{
public:
    CalibrateAxis(const rclcpp::NodeOptions &options);

private:
    Mode mMode;
    bool mHoldScheduled;

    rex_interfaces::msg::VescMotorCommand mFrameToSend;
    rclcpp::TimerBase::SharedPtr mFrameSender;

    std::array<VESC_Id_t, 4> mCalibrationMotors;
    std::map<VESC_Id_t, MotorStatusStamped> mMotorStatuses;
    rclcpp::TimerBase::SharedPtr mVelocityTimeoutTimer;

    std::map<std::string, float> mFloatParams;
    std::map<std::string, int> mIntParams;

    VESC_Id_t mCurrentMotorID;

    rex_interfaces::msg::RoverStatus::ConstSharedPtr mLastRoverStatus;
    rex_interfaces::msg::BatteryInfo::ConstSharedPtr mLastBatteryInfo;

    rclcpp::Publisher<rex_interfaces::msg::VescMotorCommand>::SharedPtr mCalibrationMotorCommandPub;
    rclcpp::Subscription<rex_interfaces::msg::VescStatus>::SharedPtr mVescStatusSub;
    rclcpp::Subscription<rex_interfaces::msg::CalibrateAxis>::SharedPtr mCalibrateAxisSub;
    rclcpp::Subscription<rex_interfaces::msg::RoverStatus>::SharedPtr mRoverStatusSub;
    rclcpp::Subscription<rex_interfaces::msg::BatteryInfo>::SharedPtr mBatteryInfoSub;

    void initParams();
    rclcpp::node_interfaces::PostSetParametersCallbackHandle::SharedPtr mParamCallbackHandle;

    // ######################### MSG HANDLERS #########################
    void handleVescStatus(const rex_interfaces::msg::VescStatus::ConstSharedPtr &msg);
    void handleCalibrateAxis(const rex_interfaces::msg::CalibrateAxis::ConstSharedPtr &msg);
    void handleRoverStatus(const rex_interfaces::msg::RoverStatus::ConstSharedPtr &msg);
    void handleBatteryInfo(const rex_interfaces::msg::BatteryInfo::ConstSharedPtr &msg);

    // ######################### MODES #########################
    void modeNothing();
    void modeSetPos(VESC_Id_t vescID, float pos);
    void modeSetVelocity(VESC_Id_t vescID, float velocity);
    void modeHold(VESC_Id_t vescID);

    // ######################### TIMER-RELATED #########################
    void startTimeout();
    void cancelTimeout();
    void timeoutTimerTick();

    void stopMotor(VESC_Id_t vescID);

    // ######################### CAN FRAMES #########################
    rex_interfaces::msg::VescMotorCommand frameStop(VESC_Id_t vescID);
    rex_interfaces::msg::VescMotorCommand frameSetOrigin(VESC_Id_t vescID);
    rex_interfaces::msg::VescMotorCommand frameSetPosition(VESC_Id_t vescID, float position);
    rex_interfaces::msg::VescMotorCommand frameSetVelocity(VESC_Id_t vescID, float speed);

    void sendFrame();

    // ######################### UTILITY #########################
    bool calibrationMotorsContains(VESC_Id_t vescID);
    bool checkSetPosEndCondition(const rex_interfaces::msg::VescStatus::ConstSharedPtr &msg);
    bool isTimestampOutdated(rclcpp::Time stamp);
    bool isRecordedStatusValid(VESC_Id_t vescID);
};

template <typename T>
int signum(T val);

#endif // CALIBRATE_AXIS_H