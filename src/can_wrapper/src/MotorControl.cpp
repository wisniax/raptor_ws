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
	can_msgs::Frame can_msg1 = encodeMotorVel(msg.frontLeft.setValue, static_cast<VescCan::Consts::Command>(msg.frontLeft.commandId), RosCanConstants::VescIds::front_left);
	mRawCanPub.publish(can_msg1);
	can_msgs::Frame can_msg2 = encodeMotorVel(msg.frontRight.setValue, static_cast<VescCan::Consts::Command>(msg.frontRight.commandId), RosCanConstants::VescIds::front_right);
	mRawCanPub.publish(can_msg2);
	// can_msgs::Frame can_msg3 = encodeMotorVel(msg.rearLeft, msg.commandId, RosCanConstants::VescIds::rear_left);
	// mRawCanPub.publish(can_msg3);
	// can_msgs::Frame can_msg4 = encodeMotorVel(msg.rearRight, msg.commandId, RosCanConstants::VescIds::rear_right);
	// mRawCanPub.publish(can_msg4);
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