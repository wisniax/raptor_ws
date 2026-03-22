#include <can_bridge/BatteryInfoForwarder.hpp>

BatteryInfoForwarder::BatteryInfoForwarder(rclcpp::Node::SharedPtr &nh) : mNh(nh)
{
	const rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(256));
	mBatteryInfoGrabber = mNh->create_subscription<can_msgs::msg::Frame>(
		RosCanConstants::RosTopics::can_raw_RX, qos,
		std::bind(&BatteryInfoForwarder::batteryInfoGrabber, this, std::placeholders::_1));
	mBatteryInfoPublisher = mNh->create_publisher<rex_interfaces::msg::BatteryInfo>(
		RosCanConstants::RosTopics::can_battery_info, qos);
}

void BatteryInfoForwarder::batteryInfoGrabber(const can_msgs::msg::Frame::ConstSharedPtr &frame)
{
	auto rf = VescInterop::rosToVesc(*frame);

	switch (rf.command)
	{
	case VESC_COMMAND_STATUS_9:
		VESC_Status_9 status9;
		VESC_ZeroMemory(&status9, sizeof(status9));
		VESC_convertRawToStatus9(&status9, &rf);
		newVescBatteryInfo(status9);
		break;
	default:
		return;
	};

	batteryInfoPublisher();
}

void BatteryInfoForwarder::newVescBatteryInfo(VESC_Status_9 &status9)
{
	mBatteryInfo = status9;
	mBatteryInfoFresh = true;
}

void BatteryInfoForwarder::batteryInfoPublisher()
{
	if (!mBatteryInfoFresh)
		return;

	rex_interfaces::msg::BatteryInfo status;

	status.id = mBatteryInfo.vescID;
	status.slot = mBatteryInfo.vescID & 0xF;

	status.voltage = mBatteryInfo.voltage;
	status.current = mBatteryInfo.current;
	status.temperature = mBatteryInfo.temperature;
	status.charge_percent = mBatteryInfo.charge;
	status.battery_status = mBatteryInfo.batteryStatus;
	status.hotswap_status = mBatteryInfo.hotswapStatus;

	status.header.stamp = rclcpp::Clock().now();

	mBatteryInfoPublisher->publish(status);

	if (mBatteryInfoFresh)
		mBatteryInfoFresh = false;
}