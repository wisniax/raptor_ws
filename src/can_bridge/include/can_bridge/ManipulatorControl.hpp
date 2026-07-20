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

extern "C"
{
#include <libVescCan/VESC.h>
}

using ManipulatorControlMsg = rex_interfaces::msg::ManipulatorControl;
using RoverStatusMsg = rex_interfaces::msg::RoverStatus;

class ManipulatorControl : public rclcpp::Node
{
public:
	ManipulatorControl(const rclcpp::NodeOptions & options);

private:
    bool isManipulatorMode(const RoverStatusMsg::ConstSharedPtr &msg);
    void handleRoverStatusClb(const RoverStatusMsg::ConstSharedPtr &roverStatusMsg);
    void handleManipulatorCtl(const rex_interfaces::msg::ManipulatorControl::ConstSharedPtr &manipulatorCtlMsg);

	can_msgs::msg::Frame encodeStepper(const rex_interfaces::msg::VescMotorCommand &stepper, const VESC_Id_t vescId);

	rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr mRawCanPub;							 /**< ROS publisher for raw CAN messages. */
	rclcpp::Subscription<rex_interfaces::msg::ManipulatorControl>::SharedPtr mManipulatorCtlSub; /**< ROS subscriber for motor velocity messages. */
    rclcpp::Subscription<RoverStatusMsg>::SharedPtr mRoverStatusSub;   /**< ROS subscriber for SamplerControl messages. */

    RoverStatusMsg::ConstSharedPtr mRoverStatusMsgLast;
};

#endif // ManipulatorControl_h_
