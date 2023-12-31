#include "can_wrapper/MotorControl.hpp"

MotorControl::MotorControl(float rpmScale, CM_SetMotorVel_ContMode mode)
{
	setRPMscale(rpmScale);
	setControlMode(mode);

	mRawCanPub = mNh.advertise<can_msgs::Frame>(RosCanConstants::RosTopics::can_raw_TX, 256);
	mSetMotorVelSub = mNh.subscribe(RosCanConstants::RosTopics::can_set_motor_vel, 256, &MotorControl::handleSetMotorVel, this);
}

void MotorControl::setRPMscale(float rpmScale)
{
	mRPM_scale = rpmScale;
}

void MotorControl::setMotorVel(const float msg[], const CM_Address_t adr){
	mRawCanPub.publish(encodeMotorVel(msg, adr));
}

void MotorControl::setControlMode(CM_SetMotorVel_ContMode mode)
{
	mControlMode = mode;
}

void MotorControl::handleSetMotorVel(const can_wrapper::Wheels &msg)
{
	sendMotorVel(msg);
}

void MotorControl::sendMotorVel(const can_wrapper::Wheels msg)
{
	float leftMotors[] = {msg.frontLeft, msg.midLeft, msg.rearLeft};
	float rightMotors[] = {msg.frontRight, msg.midRight, msg.rearRight};

	can_msgs::Frame can_msg = encodeMotorVel(leftMotors, CM_ADDRESS_STM_LEFT);
	mRawCanPub.publish(can_msg);
	can_msgs::Frame can_msg2 = encodeMotorVel(rightMotors, CM_ADDRESS_STM_RIGHT);
	mRawCanPub.publish(can_msg2);
}

can_msgs::Frame MotorControl::encodeMotorVel(const float msg[], const CM_Address_t adr)
{
	CM_CanMessage cm;
	cm.address = CM_ADDRESS_FAMILY_MOTOR_CONTROL | adr;
	cm.dataLen = 5;
	cm.data.mode.cont_mode = mControlMode;
	cm.data.mode.reason = 0;

	float rpmScale = (mControlMode == CM_SETMOTORVEL_CONTMODE_PWM) ? CM_MOTORVEL_VALUE_MAX : mRPM_scale;

	cm.data.set_motor_vel.motor_A_value = static_cast<uint16_t>(std::abs(msg[0]) * rpmScale);
	cm.data.set_motor_vel.motor_A_dir = msg[0] >= 0 ? 0 : 1;
	cm.data.set_motor_vel.motor_B_value = static_cast<uint16_t>(std::abs(msg[1]) * rpmScale);
	cm.data.set_motor_vel.motor_B_dir = msg[1] >= 0 ? 0 : 1;
	cm.data.set_motor_vel.motor_C_value = static_cast<uint16_t>(std::abs(msg[2]) * rpmScale);
	cm.data.set_motor_vel.motor_C_dir = msg[2] >= 0 ? 0 : 1;

	can_msgs::Frame fr = CM_convert_CanMessage_mtof(cm);
	fr.header.seq = mSetMotorVelSeq++;
	return fr;
}