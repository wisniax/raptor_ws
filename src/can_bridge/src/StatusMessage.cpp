#include "can_bridge/StatusMessage.hpp"

StatusMessage::StatusMessage(rclcpp::Node::SharedPtr &nh, bool sendOnUpdate) : mNh(nh)
{
	rex_interfaces::msg::RoverStatus zeroMsg;
	zeroMsg.communication_state = 0;
	zeroMsg.control_mode = 0;
	zeroMsg.pad_connected = false;
	mLastStatus = zeroMsg;
	mSendOnUpdate = sendOnUpdate;

	const rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(256));
	mRawCanPub = nh->create_publisher<can_msgs::msg::Frame>(RosCanConstants::RosTopics::can_raw_TX, qos);
	mStatusMessageSub = nh->create_subscription<rex_interfaces::msg::RoverStatus>(
		RosCanConstants::RosTopics::mqtt_rover_status, qos,
		std::bind(&StatusMessage::handleStatusMessage, this, std::placeholders::_1));
}

void StatusMessage::handleStatusMessage(const rex_interfaces::msg::RoverStatus::ConstSharedPtr &msg)
{
	mLastStatus = *msg;
	if (mSendOnUpdate)
		sendStatusMessage(msg);
}

void StatusMessage::sendStatusMessage()
{
	can_msgs::msg::Frame rawFrame = encodeStatusMessage(mLastStatus);
	mRawCanPub->publish(rawFrame);
}

void StatusMessage::sendStatusMessage(const rex_interfaces::msg::RoverStatus::ConstSharedPtr &msg)
{
	can_msgs::msg::Frame rawFrame = encodeStatusMessage(*msg);
	mRawCanPub->publish(rawFrame);
}

can_msgs::msg::Frame StatusMessage::encodeStatusMessage(const rex_interfaces::msg::RoverStatus &msg)
{
	VESC_Status_10 msg_status_10;
	VESC_ZeroMemory(&msg_status_10, sizeof(msg_status_10));

	msg_status_10.vescID = RosCanConstants::VescIds::ros_can_host;
	msg_status_10.communicationState = (VESC_Status_10_CommunicationState)msg.communication_state;
	msg_status_10.flags = msg.pad_connected ? VESC_STATUS_10_FLAGS_MASK_PADCONNECTED : 0;
	msg_status_10.controlMode = (VESC_Status_10_ControlMode)msg.control_mode;

	VESC_RawFrame rf;
	VESC_ZeroMemory(&rf, sizeof(rf));
	VESC_convertStatus10ToRaw(&rf, &msg_status_10);

	can_msgs::msg::Frame fr = VescInterop::vescToRos(rf);
	return fr;
}