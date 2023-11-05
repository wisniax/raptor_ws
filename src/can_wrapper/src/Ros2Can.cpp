#include "can_wrapper/Ros2Can.hpp"

std::unique_ptr<Ros2Can> Ros2Can::instance(nullptr);

Ros2Can *Ros2Can::getSingleton()
{
	if (instance.get() == nullptr)
		instance.reset(new Ros2Can);
	return instance.get();
}

void Ros2Can::init(std::string can_TX_topic, float rpm_scale)
{
	mRPM_scale = rpm_scale;
	mRawCanPub = nh.advertise<can_msgs::Frame>(can_TX_topic + "raw", 256);

	mDriversLeft = nh.subscribe(can_TX_topic + "SetVelLeft", 256, handleSetLeftDriverVel);
	mDriversRight = nh.subscribe(can_TX_topic + "SetVelRight", 256, handleSetRightDriverVel);
	mArm123 = nh.subscribe(can_TX_topic + "SetArm123Vel", 256, handleSetArm123Vel);
	mArm456 = nh.subscribe(can_TX_topic + "SetArm456Vel", 256, handleSetArm456Vel);
}

void Ros2Can::handleSetLeftDriverVel(const geometry_msgs::Vector3Stamped &msg)
{
	instance.get()->sendMotorVel(msg, CanMessage::Address::TX_DriversLeft);
}

void Ros2Can::handleSetRightDriverVel(const geometry_msgs::Vector3Stamped &msg)
{
	instance.get()->sendMotorVel(msg, CanMessage::Address::TX_DriversRight);
}

void Ros2Can::handleSetArm123Vel(const geometry_msgs::Vector3Stamped &msg)
{
	instance.get()->sendMotorVel(msg, CanMessage::Address::TX_ArmAxis123);
}

void Ros2Can::handleSetArm456Vel(const geometry_msgs::Vector3Stamped &msg)
{
	instance.get()->sendMotorVel(msg, CanMessage::Address::TX_ArmAxis456);
}

can_msgs::Frame Ros2Can::canMessage2Frame(CanMessage cm)
{
	can_msgs::Frame fr;
	fr.id = (uint32_t)cm.address;
	fr.dlc = cm.dataLen;
	for (int i = 0; i < cm.dataLen; i++)
		fr.data[i] = cm.data.raw[i];
	return fr;
}

void Ros2Can::sendMotorVel(const geometry_msgs::Vector3Stamped msg, const CanMessage::Address adr)
{
	can_msgs::Frame can_msg = encodeMotorVel(msg, adr);
	mRawCanPub.publish(can_msg);
}

can_msgs::Frame Ros2Can::encodeMotorVel(const geometry_msgs::Vector3Stamped msg, const CanMessage::Address adr)
{
	CanMessage cm;
	cm.address = adr;
	cm.dataLen = 5;
	cm.data.mode.cont_mode = CanMessage::set_motor_vel_t::mode_cont_mode::TargetModeRpm;
	cm.data.mode.reason = 0;
	cm.data.set_motor_vel.motor_A_vel = (uint16_t)(std::abs(msg.vector.x) * mRPM_scale);
	cm.data.get_motor_vel.motor_A_dir = msg.vector.x >= 0 ? 0 : 1;
	cm.data.set_motor_vel.motor_B_vel = (uint16_t)(std::abs(msg.vector.y) * mRPM_scale);
	cm.data.get_motor_vel.motor_B_dir = msg.vector.y >= 0 ? 0 : 1;
	cm.data.set_motor_vel.motor_C_vel = (uint16_t)(std::abs(msg.vector.z) * mRPM_scale);
	cm.data.get_motor_vel.motor_C_dir = msg.vector.z >= 0 ? 0 : 1;
	can_msgs::Frame fr = canMessage2Frame(cm);
	return fr;
}

uint16_t Ros2Can::setPwmOnMotor(const geometry_msgs::Vector3Stamped msg, const CanMessage::Address adr)
{
	CanMessage cm;
	cm.address = adr;
	cm.dataLen = 5;
	cm.data.mode.cont_mode = CanMessage::set_motor_vel_t::mode_cont_mode::TargetModePwm;
	cm.data.mode.reason = 0;
	cm.data.set_motor_vel.motor_A_vel = (uint16_t)(std::abs(msg.vector.x) * 2047);
	cm.data.get_motor_vel.motor_A_dir = msg.vector.x >= 0 ? 0 : 1;
	cm.data.set_motor_vel.motor_B_vel = (uint16_t)(std::abs(msg.vector.y) * 2047);
	cm.data.get_motor_vel.motor_B_dir = msg.vector.y >= 0 ? 0 : 1;
	cm.data.set_motor_vel.motor_C_vel = (uint16_t)(std::abs(msg.vector.z) * 2047);
	cm.data.get_motor_vel.motor_C_dir = msg.vector.z >= 0 ? 0 : 1;

	can_msgs::Frame fr = canMessage2Frame(cm);
	mRawCanPub.publish(fr);
	return 0;
}