#include "can_wrapper/StatusMessage.hpp"

StatusMessage::StatusMessage(const ros::NodeHandle &nh, bool sendOnUpdate) : mNh(nh)
{
	mSendOnUpdate = sendOnUpdate;
	mStatusMessageSeq = 0;
	mRawCanPub = mNh.advertise<can_msgs::Frame>(RosCanConstants::RosTopics::can_raw_TX, 256);
	mStatusMessageSub = mNh.subscribe("/MQTT/RoverStatus", 256, &StatusMessage::handleStatusMessage, this);
}

void StatusMessage::handleStatusMessage(const can_wrapper::RoverStatus &msg)
{
	mLastStatus = msg;
	if (mSendOnUpdate)
		sendStatusMessage(msg);
}

void StatusMessage::sendStatusMessage()
{
	can_msgs::Frame rawFrame = encodeStatusMessage(mLastStatus);
	mRawCanPub.publish(rawFrame);
}

void StatusMessage::sendStatusMessage(const can_wrapper::RoverStatus &msg)
{
	can_msgs::Frame rawFrame = encodeStatusMessage(msg);
	mRawCanPub.publish(rawFrame);
}

can_msgs::Frame StatusMessage::encodeStatusMessage(const can_wrapper::RoverStatus &msg)
{
	VESC_Status_10 msg_status_10;
	VESC_ZeroMemory(&msg_status_10, sizeof(msg_status_10));

	msg_status_10.vescID = RosCanConstants::VescIds::ros_can_host;
	msg_status_10.communicationState = (VESC_Status_10_CommunicationState)msg.CommunicationState;
	msg_status_10.flags = msg.PadConnected ? VESC_STATUS_10_FLAGS_MASK_PADCONNECTED : 0;
	msg_status_10.controlMode = (VESC_Status_10_ControlMode)msg.ControlMode;

	VESC_RawFrame rf;
	VESC_ZeroMemory(&rf, sizeof(rf));
	VESC_convertStatus10ToRaw(&rf, &msg_status_10);

	can_msgs::Frame fr = VescInterop::vescToRos(rf);
	fr.header.seq = mStatusMessageSeq++;
	return fr;
}