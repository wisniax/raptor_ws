#include <can_bridge/ProbeStatusForwarder.hpp>

ProbeStatusForwarder::ProbeStatusForwarder(rclcpp::Node::SharedPtr &nh) : mNh(nh)
{
	const rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(256));
	mProbeStatusGrabber = mNh->create_subscription<can_msgs::msg::Frame>(
		RosCanConstants::RosTopics::can_raw_RX, qos,
		std::bind(&ProbeStatusForwarder::probeStatusGrabber, this, std::placeholders::_1));
	mProbeStatusPublisher = mNh->create_publisher<rex_interfaces::msg::ProbeStatus>(
		RosCanConstants::RosTopics::can_probe_status, qos);
}

void ProbeStatusForwarder::probeStatusGrabber(const can_msgs::msg::Frame::ConstSharedPtr &frame)
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
	case VESC_COMMAND_STATUS_9:
		VESC_Status_9 status9;
		VESC_ZeroMemory(&status9, sizeof(status9));
		VESC_convertRawToStatus9(&status9, &rf);
		newStatus9(status9);
		break;
	default:
		return;
	};

	statusPublisher();
}

void ProbeStatusForwarder::newStatus8(VESC_Status_8 &status8)
{
	mStatus8 = status8;
	mStatus8Fresh = true;
	mStatus8Staleness = 0;

	if (mStatus9Staleness < UINT8_MAX)
		mStatus9Staleness++;
}

void ProbeStatusForwarder::newStatus9(VESC_Status_9 &status9)
{
	mStatus9 = status9;
	mStatus9Fresh = true;
	mStatus9Staleness = 0;

	if (mStatus8Staleness < UINT8_MAX)
		mStatus8Staleness++;
}

void ProbeStatusForwarder::warnRotten(int statusNum)
{
	if (!mRottenNoted)
	{
		mRottenNoted = true;
		RCLCPP_WARN(mNh->get_logger(), "Status %i was not updated recently while other was (Threshold (%i) reached). Publishing of ProbeStatus will resume with last value!", statusNum, ROTTEN_THRESHOLD);
	}
}

void ProbeStatusForwarder::infoNotRotten()
{
	if (mRottenNoted)
	{
		mRottenNoted = false;
		RCLCPP_DEBUG(mNh->get_logger(), "[ProbeStatusForwarder]: All statuses were updated recently.");
	}
}

void ProbeStatusForwarder::statusPublisher()
{
	if (mStatus8Staleness > ROTTEN_THRESHOLD)
	{
		warnRotten(8);
	}
	else if (mStatus9Staleness > ROTTEN_THRESHOLD)
	{
		warnRotten(9);
	}
	else if (!mStatus8Fresh || !mStatus9Fresh)
		return;

	rex_interfaces::msg::ProbeStatus status;

	status.weight_a = mStatus8.weightA;
	status.distance = mStatus8.distance;
	status.humidity = mStatus8.humidity;
	status.vibrations = mStatus8.vibrations;
	status.weight_b = mStatus8.weightB;

	status.potassium = mStatus9.potassium;
	status.nitrogen = mStatus9.nitrogen;
	status.phosphorus = mStatus9.phosphorus;
	status.ph = mStatus9.ph;

	status.header.stamp = rclcpp::Clock().now();

	mProbeStatusPublisher->publish(status);

	if (mStatus8Fresh && mStatus9Fresh)
	{
		infoNotRotten();
		mStatus8Fresh = false;
		mStatus9Fresh = false;
	}
}