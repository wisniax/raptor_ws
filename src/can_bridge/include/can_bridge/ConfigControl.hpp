#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "rclcpp/rclcpp.hpp"
#include <rclcpp_components/register_node_macro.hpp>
#include <string>
#include <memory>
#include <array>

#include "can_bridge/VescInterop.hpp"
#include "can_bridge/RosCanConstants.hpp"

#include <can_msgs/msg/frame.hpp>
#include "rex_interfaces/msg/vesc_motor_command.hpp"
#include "rex_interfaces/msg/rover_status.hpp"
#include "rex_interfaces/msg/battery_info.hpp"

extern "C"
{
#include <libVescCan/VESC.h>
}

using VescMotorMsg = rex_interfaces::msg::VescMotorCommand;
using RoverStatusMsg = rex_interfaces::msg::RoverStatus;
using BatteryInfoMsg = rex_interfaces::msg::BatteryInfo;

/**
 * @brief Class for interfacing ROS with CAN bus.
 */
class ConfigControl : public rclcpp::Node
{
public:
    ConfigControl(const rclcpp::NodeOptions & options);

private:
    void handleCalibrationMotorCommand(const VescMotorMsg::ConstSharedPtr &msg);
    void handleRoverStatus(const RoverStatusMsg::ConstSharedPtr &msg);
    void handleBatteryInfo(const BatteryInfoMsg::ConstSharedPtr &msg);

    bool isConfigMode();

    can_msgs::msg::Frame encodeMotorVel(const VescMotorMsg &vescMotorCommand, const VESC_Id_t vescId);

    rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr mRawCanPub;				  /**< ROS2 publisher for raw CAN messages. */
    rclcpp::Subscription<VescMotorMsg>::SharedPtr mCalibrationMotorCommandSub;
	rclcpp::Subscription<RoverStatusMsg>::SharedPtr mRoverStatusSub;
    rclcpp::Subscription<BatteryInfoMsg>::SharedPtr mBatteryInfoSub;

    RoverStatusMsg::ConstSharedPtr mLastRoverStatus;
    BatteryInfoMsg::ConstSharedPtr mLastBatteryInfo;
};

#endif // MOTOR_CONTROL_H
