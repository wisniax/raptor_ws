#ifndef ProbeStatusForwarder_h_
#define ProbeStatusForwarder_h_

#include "rclcpp/rclcpp.hpp"
#include <boost/shared_ptr.hpp>

#include <can_bridge/RosCanConstants.hpp>
#include <can_bridge/VescInterop.hpp>

#include <can_msgs/msg/frame.hpp>
#include <can_bridge/msg/probe_status.hpp>

extern "C"
{
#include <libVescCan/VESC.h>
}

class ProbeStatusForwarder
{
public:
	ProbeStatusForwarder(rclcpp::Node::SharedPtr &nh);

private:
	void probeStatusGrabber(const can_msgs::msg::Frame::ConstSharedPtr &frame);
	void statusPublisher();

	void newStatus8(VESC_Status_8 &status8);
	void newStatus9(VESC_Status_9 &status9);

	void warnRotten(int statusNum);
	void infoNotRotten();

	const uint8_t ROTTEN_THRESHOLD = 10;

	rclcpp::Node::SharedPtr mNh;

	bool mRottenNoted = false;

	VESC_Status_8 mStatus8;
	bool mStatus8Fresh = false;
	uint8_t mStatus8Staleness = 0;

	VESC_Status_9 mStatus9;
	bool mStatus9Fresh = false;
	uint8_t mStatus9Staleness = 0;

	rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr mProbeStatusGrabber;
	rclcpp::Publisher<can_bridge::msg::ProbeStatus>::SharedPtr mProbeStatusPublisher;
};

#endif // ProbeStatusForwarder_h_