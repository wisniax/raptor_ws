#include "can_bridge/MotorControl.hpp"

MotorControl::MotorControl(rclcpp::Node::SharedPtr &nh) : mNh(nh)
{
	const rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(256));

	mRawCanPub = mNh->create_publisher<can_msgs::msg::Frame>(RosCanConstants::RosTopics::can_raw_TX, qos);
	mSetMotorVelSub = mNh->create_subscription<can_bridge::msg::Wheels>(RosCanConstants::RosTopics::can_set_motor_vel, qos, std::bind(&MotorControl::handleSetMotorVel, this, std::placeholders::_1));
}

void MotorControl::handleSetMotorVel(const can_bridge::msg::Wheels::ConstSharedPtr &msg)
{
	sendMotorVel(msg);
}

void MotorControl::sendMotorVel(const can_bridge::msg::Wheels::ConstSharedPtr &msg)
{
	//8 since there are 4 wheels, each being vesc + stepper combo
	std::array<can_msgs::msg::Frame, 8> sendQueue;

	auto sendQueueIter = sendQueue.begin();

	//stepper

	*sendQueueIter++ = encodeMotorVel
	(
		msg->front_left.turn.set_value,
		static_cast<VESC_Command>(msg->front_left.turn.command_id),
		RosCanConstants::VescIds::front_left_stepper
	);

	*sendQueueIter++ = encodeMotorVel
	(
		msg->front_right.turn.set_value,
		static_cast<VESC_Command>(msg->front_right.turn.command_id),
		RosCanConstants::VescIds::front_right_stepper
	);

	*sendQueueIter++ = encodeMotorVel
	(
		msg->rear_left.turn.set_value,
		static_cast<VESC_Command>(msg->rear_left.turn.command_id),
		RosCanConstants::VescIds::rear_left_stepper
	);

	*sendQueueIter++ = encodeMotorVel
	(
		msg->rear_right.turn.set_value,
		static_cast<VESC_Command>(msg->rear_right.turn.command_id),
		RosCanConstants::VescIds::rear_right_stepper
	);

	//vesc

	*sendQueueIter++ = encodeMotorVel
	(
		msg->front_left.drive.set_value,
		static_cast<VESC_Command>(msg->front_left.drive.command_id),
		RosCanConstants::VescIds::front_left_vesc
	);

	*sendQueueIter++ = encodeMotorVel
	(
		msg->front_right.drive.set_value,
		static_cast<VESC_Command>(msg->front_right.drive.command_id),
		RosCanConstants::VescIds::front_right_vesc
	);

	*sendQueueIter++ = encodeMotorVel
	(
		msg->rear_left.drive.set_value,
		static_cast<VESC_Command>(msg->rear_left.drive.command_id),
		RosCanConstants::VescIds::rear_left_vesc
	);

	*sendQueueIter++ = encodeMotorVel
	(
		msg->rear_right.drive.set_value,
		static_cast<VESC_Command>(msg->rear_right.drive.command_id),
		RosCanConstants::VescIds::rear_right_vesc
	);

	//send it ALL

	for(auto iter = sendQueue.begin(); iter < sendQueue.end(); iter++)
		mRawCanPub->publish(*iter);
}

can_msgs::msg::Frame MotorControl::encodeMotorVel(const float msg, const VESC_Command command, const VESC_Id_t vescId)
{
	VESC_CommandFrame cmdf;
	VESC_ZeroMemory(&cmdf, sizeof(cmdf));

	cmdf.commandData = msg;
	cmdf.command = command;
	cmdf.vescID = vescId;

	VESC_RawFrame rf;
	VESC_ZeroMemory(&rf, sizeof(rf));
	VESC_convertCmdToRaw(&rf, &cmdf);

	can_msgs::msg::Frame fr = VescInterop::vescToRos(rf);
	fr.header.stamp = rclcpp::Clock().now();

	return fr;
}