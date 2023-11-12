#include "can_wrapper/Can2Ros.hpp"

std::unique_ptr<Can2Ros> Can2Ros::instance(nullptr);

Can2Ros *Can2Ros::getSingleton()
{
	if (instance.get() == nullptr)
		instance.reset(new Can2Ros);
	return instance.get();
}

void Can2Ros::init(float rpm_scale)
{
	mRPM_scale = rpm_scale;
	mRawCanSub = nh.subscribe(RosCanConstants::RosTopics::can_raw_RX, 256, handleRosCallback);
	mRealMotorVelPub = nh.advertise<can_wrapper::Wheels>(RosCanConstants::RosTopics::can_get_motor_vel, 128);
}

void Can2Ros::handleRosCallback(const can_msgs::Frame::ConstPtr &msg)
{
	CanMessage cm(msg.get());
	instance.get()->handleFrame(cm);
}

void Can2Ros::handleFrame(CanMessage cm)
{
	geometry_msgs::Point32 vec;
	switch (cm.address)
	{
	case CanMessage::Address::RX_DriversLeft:
		vec = decodeMotorVel(cm);
		mWheelsVel.frontLeft = vec.x;
		mWheelsVel.midLeft = vec.y;
		mWheelsVel.rearLeft = vec.z;
		tryPublishWheelsVel();
		break;
	case CanMessage::Address::RX_DriversRight:
		vec = decodeMotorVel(cm);
		mWheelsVel.frontRight = vec.x;
		mWheelsVel.midRight = vec.y;
		mWheelsVel.rearRight = vec.z;
		tryPublishWheelsVel();
		break;
	case CanMessage::Address::RX_ArmAxis123: // Work in progress
		break;
	case CanMessage::Address::RX_ArmAxis456: // Work in progress
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
		vec.x = (float)(cm.data.get_motor_vel.motor_A_vel) / (cm.data.get_motor_vel.motor_A_dir ? -loc_scale : loc_scale);
		vec.y = (float)(cm.data.get_motor_vel.motor_B_vel) / (cm.data.get_motor_vel.motor_B_dir ? -loc_scale : loc_scale);
		vec.z = (float)(cm.data.get_motor_vel.motor_C_vel) / (cm.data.get_motor_vel.motor_C_dir ? -loc_scale : loc_scale);
		break;
	}
	case 2:
	case 3:
	default:
		break;
	}
	return vec;
}

void Can2Ros::tryPublishWheelsVel()
{
	if (mWheelsVel.header.stamp < ros::Time::now() - RosCanConstants::max_stm_sync_time)
	{
		mWheelsVel.header.stamp = ros::Time::now();
		ROS_WARN_COND(!mWasMotorVelPublishedSinceWheelsVelStampChange, "CAN: \"Wheels velocity feedback\" sync time exceeded. Frame dropped.");
		mWasMotorVelPublishedSinceWheelsVelStampChange = false;
		return;
	}
	mWheelsVel.header.seq++;
	mWasMotorVelPublishedSinceWheelsVelStampChange = true;
	mRealMotorVelPub.publish(mWheelsVel);
}