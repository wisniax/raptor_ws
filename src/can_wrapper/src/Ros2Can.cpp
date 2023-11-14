#include "can_wrapper/Ros2Can.hpp"

// static members init
bool Ros2Can::sIsInitialized = false;
ros::NodeHandle Ros2Can::sNh;
float Ros2Can::sRPM_scale = 0;
uint32_t Ros2Can::sSetMotorVelSeq = 0;

CanMessage::set_motor_vel_t::mode_cont_mode Ros2Can::sControlMode;

ros::Publisher Ros2Can::sRawCanPub;
ros::Subscriber Ros2Can::sSetMotorVelSub;

void Ros2Can::init(float rpmScale, CanMessage::set_motor_vel_t::mode_cont_mode mode)
{
	if (sIsInitialized)
		return;
	sIsInitialized = true;
	setRPMscale(rpmScale);
	setControlMode(mode);

	sRawCanPub = sNh.advertise<can_msgs::Frame>(RosCanConstants::RosTopics::can_raw_TX, 256);
	sSetMotorVelSub = sNh.subscribe(RosCanConstants::RosTopics::can_set_motor_vel, 256, handleSetMotorVel);
}

void Ros2Can::setRPMscale(float rpmScale)
{
	sRPM_scale = rpmScale;
}

void Ros2Can::setControlMode(CanMessage::set_motor_vel_t::mode_cont_mode mode)
{
	sControlMode = mode;
}

void Ros2Can::handleSetMotorVel(const can_wrapper::Wheels &msg)
{
	sendMotorVel(msg);
}

void Ros2Can::sendMotorVel(const can_wrapper::Wheels msg)
{
	float leftMotors[] = {msg.frontLeft, msg.midLeft, msg.rearLeft};
	float rightMotors[] = {msg.frontRight, msg.midRight, msg.rearRight};

	can_msgs::Frame can_msg = encodeMotorVel(leftMotors, CanMessage::Address::TX_DriversLeft);
	sRawCanPub.publish(can_msg);
	can_msgs::Frame can_msg2 = encodeMotorVel(rightMotors, CanMessage::Address::TX_DriversRight);
	sRawCanPub.publish(can_msg2);
	ros::spinOnce();
}

can_msgs::Frame Ros2Can::encodeMotorVel(const float msg[], const CanMessage::Address adr)
{
	CanMessage cm;
	cm.address = adr;
	cm.dataLen = 5;
	cm.data.mode.cont_mode = sControlMode;
	cm.data.mode.reason = 0;
	float rpmScale = (sControlMode == CanMessage::set_motor_vel_t::mode_cont_mode::TargetModePwm) ? 2047 : sRPM_scale;

	cm.data.set_motor_vel.motor_A_vel = static_cast<uint16_t>(std::abs(msg[0]) * rpmScale);
	cm.data.get_motor_vel.motor_A_dir = msg[0] >= 0 ? 0 : 1;
	cm.data.set_motor_vel.motor_B_vel = static_cast<uint16_t>(std::abs(msg[1]) * rpmScale);
	cm.data.get_motor_vel.motor_B_dir = msg[1] >= 0 ? 0 : 1;
	cm.data.set_motor_vel.motor_C_vel = static_cast<uint16_t>(std::abs(msg[2]) * rpmScale);
	cm.data.get_motor_vel.motor_C_dir = msg[2] >= 0 ? 0 : 1;
	can_msgs::Frame fr = (can_msgs::Frame)cm;
	fr.header.seq = sSetMotorVelSeq++;
	return fr;
}