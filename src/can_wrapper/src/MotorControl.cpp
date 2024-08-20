#include "can_wrapper/MotorControl.hpp"

MotorControl::MotorControl(const ros::NodeHandle &nh) : mNh(nh)
{
	mRawCanPub = mNh.advertise<can_msgs::Frame>(RosCanConstants::RosTopics::can_raw_TX, 256);
	mSetMotorVelSub = mNh.subscribe(RosCanConstants::RosTopics::can_set_motor_vel, 256, &MotorControl::handleSetMotorVel, this);
}

void MotorControl::handleSetMotorVel(const can_wrapper::Wheels &msg)
{
	sendMotorVel(msg);
}

void MotorControl::sendMotorVel(const can_wrapper::Wheels msg)
{
	//8 since there are 4 wheels, each being vesc + stepper combo
	std::array<can_msgs::Frame, 8> sendQueue;

	auto sendQueueIter = sendQueue.begin();

	//stepper

	*sendQueueIter++ = encodeMotorVel
	(
		msg.frontLeft.setAngle,
		static_cast<VESC_Command>(msg.frontLeft.commandIdAngle),
		RosCanConstants::VescIds::front_left_stepper
	);

	*sendQueueIter++ = encodeMotorVel
	(
		msg.frontRight.setAngle,
		static_cast<VESC_Command>(msg.frontRight.commandIdAngle),
		RosCanConstants::VescIds::front_right_stepper
	);

	*sendQueueIter++ = encodeMotorVel
	(
		msg.rearLeft.setAngle,
		static_cast<VESC_Command>(msg.rearLeft.commandIdAngle),
		RosCanConstants::VescIds::rear_left_stepper
	);

	*sendQueueIter++ = encodeMotorVel
	(
		msg.rearRight.setAngle,
		static_cast<VESC_Command>(msg.rearRight.commandIdAngle),
		RosCanConstants::VescIds::rear_right_stepper
	);

	//vesc

	*sendQueueIter++ = encodeMotorVel
	(
		msg.frontLeft.setValue,
		static_cast<VESC_Command>(msg.frontLeft.commandId),
		RosCanConstants::VescIds::front_left_vesc
	);

	*sendQueueIter++ = encodeMotorVel
	(
		msg.frontRight.setValue,
		static_cast<VESC_Command>(msg.frontRight.commandId),
		RosCanConstants::VescIds::front_right_vesc
	);

	*sendQueueIter++ = encodeMotorVel
	(
		msg.rearLeft.setValue,
		static_cast<VESC_Command>(msg.rearLeft.commandId),
		RosCanConstants::VescIds::rear_left_vesc
	);

	*sendQueueIter++ = encodeMotorVel
	(
		msg.rearRight.setValue,
		static_cast<VESC_Command>(msg.rearRight.commandId),
		RosCanConstants::VescIds::rear_right_vesc
	);

	//send it ALL

	for(auto iter = sendQueue.begin(); iter < sendQueue.end(); iter++)
		mRawCanPub.publish(*iter);
}

can_msgs::Frame MotorControl::encodeMotorVel(const float msg, const VESC_Command command, const VESC_Id_t vescId)
{
	VESC_CommandFrame cmdf;
	VESC_ZeroMemory(&cmdf, sizeof(cmdf));

	cmdf.commandData = msg;
	cmdf.command = command;
	cmdf.vescID = vescId;

	VESC_RawFrame rf;
	VESC_ZeroMemory(&rf, sizeof(rf));
	VESC_convertCmdToRaw(&rf, &cmdf);

	can_msgs::Frame fr = VescInterop::vescToRos(rf);
	fr.header.seq = mSetMotorVelSeq++;
	return fr;
}