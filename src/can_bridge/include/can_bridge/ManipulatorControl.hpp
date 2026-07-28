#ifndef ManipulatorControl_h_
#define ManipulatorControl_h_

#include "rclcpp/rclcpp.hpp"
#include <rclcpp_components/register_node_macro.hpp>
#include <string>
#include <array>

#include <can_bridge/VescInterop.hpp>
#include "can_bridge/RosCanConstants.hpp"

#include "can_msgs/msg/frame.hpp"
#include "rex_interfaces/msg/manipulator_control.hpp"
#include "rex_interfaces/msg/rover_status.hpp"
#include "rex_interfaces/msg/battery_info.hpp"

extern "C"
{
#include <libVescCan/VESC.h>
}

using ManipulatorControlMsg = rex_interfaces::msg::ManipulatorControl;
using RoverStatusMsg = rex_interfaces::msg::RoverStatus;
using BatteryInfoMsg = rex_interfaces::msg::BatteryInfo;

class ManipulatorControl : public rclcpp::Node
{
public:
	ManipulatorControl(const rclcpp::NodeOptions & options);

private:
    void handleManipulatorCtl(const ManipulatorControlMsg::ConstSharedPtr &manipulatorCtlMsg);
    void handleRoverStatus(const RoverStatusMsg::ConstSharedPtr &roverStatusMsg);
    void handleBatteryInfo(const BatteryInfoMsg::ConstSharedPtr &msg);

    bool isManipulatorMode();

    can_msgs::msg::Frame encodeStepper(const rex_interfaces::msg::VescMotorCommand &stepper, const VESC_Id_t vescId);

	rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr mRawCanPub;							 /**< ROS publisher for raw CAN messages. */
	rclcpp::Subscription<ManipulatorControlMsg>::SharedPtr mManipulatorCtlSub; /**< ROS subscriber for motor velocity messages. */
    rclcpp::Subscription<RoverStatusMsg>::SharedPtr mRoverStatusSub;   /**< ROS subscriber for SamplerControl messages. */
    rclcpp::Subscription<BatteryInfoMsg>::SharedPtr mBatteryInfoSub;

    RoverStatusMsg::ConstSharedPtr mLastRoverStatus;
    BatteryInfoMsg::ConstSharedPtr mLastBatteryInfo;
};

#endif // ManipulatorControl_h_
