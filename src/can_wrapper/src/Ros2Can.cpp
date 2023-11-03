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
    instance.get()->encodeMotorVel(msg, CanMessage::Address::TX_DriversLeft);
}

void Ros2Can::handleSetRightDriverVel(const geometry_msgs::Vector3Stamped &msg)
{
    instance.get()->encodeMotorVel(msg, CanMessage::Address::TX_DriversRight);
}

void Ros2Can::handleSetArm123Vel(const geometry_msgs::Vector3Stamped &msg)
{
    instance.get()->encodeMotorVel(msg, CanMessage::Address::TX_ArmAxis123);
}

void Ros2Can::handleSetArm456Vel(const geometry_msgs::Vector3Stamped &msg)
{
    instance.get()->encodeMotorVel(msg, CanMessage::Address::TX_ArmAxis456);
}

can_msgs::Frame Ros2Can::encodeMotorVel(geometry_msgs::Vector3Stamped msg, CanMessage::Address adr)
{
    CanMessage cm;
    cm.address = adr;
    cm.data.set_motor_vel.motor_A_vel = (uint16_t)(msg.vector.x * mRPM_scale);
    can_msgs::Frame fr;
    return fr;
}