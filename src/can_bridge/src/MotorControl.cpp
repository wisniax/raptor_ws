#include "can_bridge/MotorControl.hpp"

MotorControl::MotorControl(rclcpp::Node::SharedPtr &nh) : mNh(nh)
{
	const rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(256));

	mRawCanPub = mNh->create_publisher<can_msgs::msg::Frame>(RosCanConstants::RosTopics::can_raw_TX, qos);
	mSetMotorVelSub = mNh->create_subscription<rex_interfaces::msg::Wheels>(RosCanConstants::RosTopics::can_set_motor_vel, qos, std::bind(&MotorControl::handleSetMotorVel, this, std::placeholders::_1));
}

void MotorControl::handleSetMotorVel(const rex_interfaces::msg::Wheels::ConstSharedPtr &msg)
{
	sendMotorVel(msg);
}

void MotorControl::sendMotorVel(const rex_interfaces::msg::Wheels::ConstSharedPtr &msg)
{
	//8 since there are 4 wheels, each being vesc + stepper combo
	std::array<can_msgs::msg::Frame, 8> sendQueue;

	auto sendQueueIter = sendQueue.begin();

	//stepper

	*sendQueueIter++ = encodeMotorVel
	(
		msg->front_left.turn,
		RosCanConstants::VescIds::front_left_stepper
	);

	*sendQueueIter++ = encodeMotorVel
	(
		msg->front_right.turn,
		RosCanConstants::VescIds::front_right_stepper
	);

	*sendQueueIter++ = encodeMotorVel
	(
		msg->rear_left.turn,
		RosCanConstants::VescIds::rear_left_stepper
	);

	*sendQueueIter++ = encodeMotorVel
	(
		msg->rear_right.turn,
		RosCanConstants::VescIds::rear_right_stepper
	);

	//vesc

	*sendQueueIter++ = encodeMotorVel
	(
		msg->front_left.drive,
		RosCanConstants::VescIds::front_left_vesc
	);

	*sendQueueIter++ = encodeMotorVel
	(
		msg->front_right.drive,
		RosCanConstants::VescIds::front_right_vesc
	);

	*sendQueueIter++ = encodeMotorVel
	(
		msg->rear_left.drive,
		RosCanConstants::VescIds::rear_left_vesc
	);

	*sendQueueIter++ = encodeMotorVel
	(
		msg->rear_right.drive,
		RosCanConstants::VescIds::rear_right_vesc
	);

	//send it ALL

	for(auto iter = sendQueue.begin(); iter < sendQueue.end(); iter++)
		mRawCanPub->publish(*iter);
}

can_msgs::msg::Frame MotorControl::encodeMotorVel(const rex_interfaces::msg::VescMotorCommand &vescMotorCommand, const VESC_Id_t vescId)
{
	VESC_CommandFrame cmdf;
	VESC_ZeroMemory(&cmdf, sizeof(cmdf));

	switch(vescMotorCommand.command_id)
	{
		case VESC_COMMAND_SET_ORIGIN:
			cmdf.commandDataExB = vescMotorCommand.set_origin_data;
			break;
		case VESC_COMMAND_SET_POS_SPEED_LOOP:
			cmdf.commandDataEx_0 = vescMotorCommand.set_pos_speed_loop_position;
			cmdf.commandDataEx_1 = vescMotorCommand.set_pos_speed_loop_speed;
			cmdf.commandDataEx_2 = vescMotorCommand.set_pos_speed_loop_acceleration;
			break;
		default:
			cmdf.commandData = vescMotorCommand.set_value;
	}

	cmdf.command = vescMotorCommand.command_id;
	cmdf.vescID = vescId;

	VESC_RawFrame rf;
	VESC_ZeroMemory(&rf, sizeof(rf));
	VESC_convertCmdToRaw(&rf, &cmdf);

	can_msgs::msg::Frame fr = VescInterop::vescToRos(rf);
	fr.header.stamp = rclcpp::Clock().now();

	return fr;
}