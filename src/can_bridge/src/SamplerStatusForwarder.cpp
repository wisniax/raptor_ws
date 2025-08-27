#include <can_bridge/SamplerStatusForwarder.hpp>

SamplerStatusForwarder::SamplerStatusForwarder(rclcpp::Node::SharedPtr &nh) : mNh(nh)
{
	const rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(256));
	mSamplerStatusGrabber = mNh->create_subscription<can_msgs::msg::Frame>(
		RosCanConstants::RosTopics::can_raw_RX, qos,
		std::bind(&SamplerStatusForwarder::samplerStatusGrabber, this, std::placeholders::_1));
	mSamplerStatusPublisher = mNh->create_publisher<rex_interfaces::msg::SamplerFeedback>(
		RosCanConstants::RosTopics::can_sampler_status, qos);
}

void SamplerStatusForwarder::samplerStatusGrabber(const can_msgs::msg::Frame::ConstSharedPtr &frame)
{
	auto rf = VescInterop::rosToVesc(*frame);

	switch (rf.command)
	{
	case VESC_COMMAND_STATUS_8:
		VESC_Status_8 status8;
		VESC_ZeroMemory(&status8, sizeof(status8));
		VESC_convertRawToStatus8(&status8, &rf);
		newStatus8(status8);
		break;
	default:
		return;
	};

	statusPublisher();
}

void SamplerStatusForwarder::newStatus8(VESC_Status_8 &status8)
{
	mStatus8 = status8;
	mStatus8Fresh = true;
}

void SamplerStatusForwarder::statusPublisher()
{
	if (!mStatus8Fresh)
		return;

	rex_interfaces::msg::SamplerFeedback status;

	status.weight_a = mStatus8.weightA;
	status.weight_b = mStatus8.weightB;
	status.weight_c = mStatus8.weightC;
	status.ph = mStatus8.ph;
	status.distance = mStatus8.distance;

	status.header.stamp = rclcpp::Clock().now();

	mSamplerStatusPublisher->publish(status);

	if (mStatus8Fresh)
		mStatus8Fresh = false;
}