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
		static_cast<VescCan::Consts::Command>(msg.frontLeft.commandIdAngle),
		RosCanConstants::VescIds::front_left_stepper
	);

	*sendQueueIter++ = encodeMotorVel
	(
		msg.frontRight.setAngle,
		static_cast<VescCan::Consts::Command>(msg.frontRight.commandIdAngle),
		RosCanConstants::VescIds::front_right_stepper
	);

	*sendQueueIter++ = encodeMotorVel
	(
		msg.rearLeft.setAngle,
		static_cast<VescCan::Consts::Command>(msg.rearLeft.commandIdAngle),
		RosCanConstants::VescIds::rear_left_stepper
	);

	*sendQueueIter++ = encodeMotorVel
	(
		msg.rearRight.setAngle,
		static_cast<VescCan::Consts::Command>(msg.rearRight.commandIdAngle),
		RosCanConstants::VescIds::rear_right_stepper
	);

	//vesc

	*sendQueueIter++ = encodeMotorVel
	(
		msg.frontLeft.setValue,
		static_cast<VescCan::Consts::Command>(msg.frontLeft.commandId),
		RosCanConstants::VescIds::front_left_vesc
	);

	*sendQueueIter++ = encodeMotorVel
	(
		msg.frontRight.setValue,
		static_cast<VescCan::Consts::Command>(msg.frontRight.commandId),
		RosCanConstants::VescIds::front_right_vesc
	);

	*sendQueueIter++ = encodeMotorVel
	(
		msg.rearLeft.setValue,
		static_cast<VescCan::Consts::Command>(msg.rearLeft.commandId),
		RosCanConstants::VescIds::rear_left_vesc
	);

	*sendQueueIter++ = encodeMotorVel
	(
		msg.rearRight.setValue,
		static_cast<VescCan::Consts::Command>(msg.rearRight.commandId),
		RosCanConstants::VescIds::rear_right_vesc
	);

	//send it ALL

	for(auto iter = sendQueue.begin(); iter < sendQueue.end(); iter++)
		mRawCanPub.publish(*iter);
}

can_msgs::Frame MotorControl::encodeMotorVel(const float msg, const VescCan::Consts::Command command, const uint8_t vescId)
{
	VescCan::CanFrame frame;
	switch (command)
	{
	case VescCan::Consts::Command::SET_DUTY:
		frame = VescCan::CanFrameFactory<VescCan::ConstsPacked::SetDuty>::create(vescId, msg);
		break;

	case VescCan::Consts::Command::SET_CURRENT:
		frame = VescCan::CanFrameFactory<VescCan::ConstsPacked::SetCurrent>::create(vescId, msg);
		break;

	case VescCan::Consts::Command::SET_CURRENT_BRAKE:
		frame = VescCan::CanFrameFactory<VescCan::ConstsPacked::SetCurrentBrake>::create(vescId, msg);
		break;

	case VescCan::Consts::Command::SET_RPM:
		frame = VescCan::CanFrameFactory<VescCan::ConstsPacked::SetRpm>::create(vescId, msg);
		break;

	case VescCan::Consts::Command::SET_POS:
		frame = VescCan::CanFrameFactory<VescCan::ConstsPacked::SetPos>::create(vescId, msg);
		break;

	case VescCan::Consts::Command::SET_CURRENT_REL:
		frame = VescCan::CanFrameFactory<VescCan::ConstsPacked::SetCurrentRel>::create(vescId, msg);
		break;

	case VescCan::Consts::Command::SET_CURRENT_BRAKE_REL:
		frame = VescCan::CanFrameFactory<VescCan::ConstsPacked::SetCurrentBreakeRel>::create(vescId, msg);
		break;

	case VescCan::Consts::Command::SET_CURRENT_HANDBRAKE:
		frame = VescCan::CanFrameFactory<VescCan::ConstsPacked::SendCurrentHandbrake>::create(vescId, msg);
		break;

	case VescCan::Consts::Command::SET_CURRENT_HANDBRAKE_REL:
		frame = VescCan::CanFrameFactory<VescCan::ConstsPacked::SendCurrentHandbrakeRel>::create(vescId, msg);
		break;

	default:
		break;
	}

	can_msgs::Frame fr = can_msgs::Frame(frame);
	fr.header.seq = mSetMotorVelSeq++;
	return fr;
}