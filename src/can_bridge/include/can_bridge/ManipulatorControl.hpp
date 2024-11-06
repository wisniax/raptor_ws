#ifndef ManipulatorControl_h_
#define ManipulatorControl_h_

#include "rclcpp/rclcpp.hpp"
#include <string>
#include "can_msgs/msg/frame.hpp"
#include <array>
#include <can_bridge/VescInterop.hpp>
#include "can_bridge/RosCanConstants.hpp"
#include "can_bridge/msg/manipulator_control.hpp"
extern "C"
{
#include <libVescCan/VESC.h>
}

class ManipulatorControl
{
public:
	ManipulatorControl(rclcpp::Node::SharedPtr &nh);

private:
	void handleManipulatorCtl(const can_bridge::msg::ManipulatorControl::ConstSharedPtr &manipulatroCtlMsg);

	can_msgs::msg::Frame encodeStepper(const can_bridge::msg::VescMotorCommand &stepper, const VESC_Id_t vescId);

	rclcpp::Node::SharedPtr mNh;

	rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr mRawCanPub;							 /**< ROS publisher for raw CAN messages. */
	rclcpp::Subscription<can_bridge::msg::ManipulatorControl>::SharedPtr mManipulatorCtlSub; /**< ROS subscriber for motor velocity messages. */
};

#endif // ManipulatorControl_h_