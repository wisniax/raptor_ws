#ifndef SamplerControl_h_
#define SamplerControl_h_

#include "rclcpp/rclcpp.hpp"
#include <string>
#include "can_msgs/msg/frame.hpp"
#include <array>
#include <can_bridge/VescInterop.hpp>
#include "can_bridge/RosCanConstants.hpp"
#include "rex_interfaces/msg/sampler_control.hpp"
extern "C"
{
#include <libVescCan/VESC.h>
}

class SamplerControl
{
public:
	SamplerControl(rclcpp::Node::SharedPtr &nh);

private:
	void handleSamplerCtl(const rex_interfaces::msg::SamplerControl::ConstSharedPtr &samplerCtlMsg);
	void doStuff();
	void publish(const VESC_CommandFrame *arr, int arr_size);

	rclcpp::Node::SharedPtr mNh;

	rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr mRawCanPub;						 /**< ROS publisher for raw CAN messages. */
	rclcpp::Subscription<rex_interfaces::msg::SamplerControl>::SharedPtr mSamplerCtlSub; /**< ROS subscriber for SamplerControl messages. */

	rclcpp::TimerBase::SharedPtr mTimerXd;

	rex_interfaces::msg::SamplerControl::ConstSharedPtr mSamplerCtlMsgLast;
};

#endif // SamplerControl_h_