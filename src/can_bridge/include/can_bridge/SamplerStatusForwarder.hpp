#ifndef SamplerStatusForwarder_h_
#define SamplerStatusForwarder_h_

#include "rclcpp/rclcpp.hpp"
#include <rclcpp_components/register_node_macro.hpp>
#include <boost/shared_ptr.hpp>

#include <can_bridge/RosCanConstants.hpp>
#include <can_bridge/VescInterop.hpp>

#include <can_msgs/msg/frame.hpp>
#include <rex_interfaces/msg/sampler_feedback.hpp>

extern "C"
{
#include <libVescCan/VESC.h>
}

class SamplerStatusForwarder : public rclcpp::Node
{
public:
	SamplerStatusForwarder(const rclcpp::NodeOptions & options);

private:
	void samplerStatusGrabber(const can_msgs::msg::Frame::ConstSharedPtr &frame);
	void statusPublisher();

	void newStatus8(VESC_Status_8 &status8);

	VESC_Status_8 mStatus8;
	bool mStatus8Fresh = false;

	rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr mSamplerStatusGrabber;
	rclcpp::Publisher<rex_interfaces::msg::SamplerFeedback>::SharedPtr mSamplerStatusPublisher;
};

#endif // SamplerStatusForwarder_h_
