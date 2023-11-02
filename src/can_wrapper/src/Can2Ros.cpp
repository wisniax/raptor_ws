#include "can_wrapper/Can2Ros.hpp"

std::unique_ptr<Can2Ros> Can2Ros::instance(nullptr);

Can2Ros *Can2Ros::getSingleton()
{
	if (instance.get() == nullptr)
		instance.reset(new Can2Ros);
	return instance.get();
}

void Can2Ros::init(std::string can_RX_topic, float rpm_scale)
{
	mRPM_scale = rpm_scale;
	
	mRawCanSub = nh.subscribe(can_RX_topic + "raw", 256, handleRosCallback);
	mDriversLeft = nh.advertise<geometry_msgs::Point32>(can_RX_topic + "EncLeft", 256);
	mDriversRight = nh.advertise<geometry_msgs::Point32>(can_RX_topic + "EncRight", 256);
	mArm123 = nh.advertise<geometry_msgs::Point32>(can_RX_topic + "EncArm123", 256);
	mArm456 = nh.advertise<geometry_msgs::Point32>(can_RX_topic + "EncArm456", 256);
}

void Can2Ros::handleRosCallback(const can_msgs::Frame::ConstPtr &msg)
{
	CanMessage cm(msg.get());
	instance.get()->handleFrame(cm);
}

void Can2Ros::handleFrame(CanMessage cm)
{
	geometry_msgs::Point32 vec = decodeMotorVel(cm);

	switch (cm.address)
	{
	case CanMessage::Address::RX_DriversLeft:
		mDriversLeft.publish(vec);
		break;
	case CanMessage::Address::RX_DriversRight:
		mDriversRight.publish(vec);
		break;
	case CanMessage::Address::RX_ArmAxis123:
		mArm123.publish(vec);
		break;
	case CanMessage::Address::RX_ArmAxis456:
		mArm456.publish(vec);
		break;
	case CanMessage::Address::Invalid:
	default:
		break;
	}
}

geometry_msgs::Point32 Can2Ros::decodeMotorVel(CanMessage cm)
{
	geometry_msgs::Point32 vec;
	switch (cm.data.mode.cont_mode)
	{
	case 0:
	case 1:
    {
		float loc_scale = (cm.data.mode.cont_mode == CanMessage::get_motor_vel_t::mode_cont_mode::FeedModeRpmNew) ? mRPM_scale * 5 : mRPM_scale;
		vec.x = (float)(cm.data.get_motor_vel.motor_A_vel) / cm.data.get_motor_vel.motor_A_dir ? -loc_scale : loc_scale;
		vec.y = (float)(cm.data.get_motor_vel.motor_B_vel) / cm.data.get_motor_vel.motor_B_dir ? -loc_scale : loc_scale;
		vec.z = (float)(cm.data.get_motor_vel.motor_C_vel) / cm.data.get_motor_vel.motor_C_dir ? -loc_scale : loc_scale;
		break;
    }
	case 2:
	case 3:
	default:
		break;
	}
	return vec;
}