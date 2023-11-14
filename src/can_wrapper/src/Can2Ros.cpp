#include "can_wrapper/Can2Ros.hpp"

bool Can2Ros::sIsInitialized = false;
ros::NodeHandle Can2Ros::sNh;
float Can2Ros::sRPM_scale = 0;
can_wrapper::Wheels Can2Ros::sWheelsVel;
bool Can2Ros::sWasMotorVelPublishedSinceWheelsVelStampChange = false;

ros::Subscriber Can2Ros::sRawCanSub;
ros::Publisher Can2Ros::sRealMotorVelPub;

void Can2Ros::init(float rpm_scale)
{
	if (sIsInitialized)
		return;
	sIsInitialized = true;
	sRPM_scale = rpm_scale;
	sRawCanSub = sNh.subscribe(RosCanConstants::RosTopics::can_raw_RX, 256, handleRosCallback);
	sRealMotorVelPub = sNh.advertise<can_wrapper::Wheels>(RosCanConstants::RosTopics::can_get_motor_vel, 128);
}

void Can2Ros::handleRosCallback(const can_msgs::Frame::ConstPtr &msg)
{
	if ((msg->id & CanMessage::Masks::Encoder_Velocity_Feedback_Mask) != CanMessage::Masks::Encoder_Velocity_Feedback_Mask)
		return;
	CanMessage cm(msg.get());
	handleFrame(cm);
}

void Can2Ros::handleFrame(CanMessage cm)
{
	geometry_msgs::Point32 vec;
	switch (cm.address)
	{
	case CanMessage::Address::RX_DriversLeft:
		vec = decodeMotorVel(cm);
		sWheelsVel.frontLeft = vec.x;
		sWheelsVel.midLeft = vec.y;
		sWheelsVel.rearLeft = vec.z;
		tryPublishWheelsVel();
		break;
	case CanMessage::Address::RX_DriversRight:
		vec = decodeMotorVel(cm);
		sWheelsVel.frontRight = vec.x;
		sWheelsVel.midRight = vec.y;
		sWheelsVel.rearRight = vec.z;
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
		float loc_scale = (cm.data.mode.cont_mode == CanMessage::get_motor_vel_t::mode_cont_mode::FeedModeRpmNew) ? sRPM_scale * 5 : sRPM_scale;
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
	if (sWheelsVel.header.stamp < ros::Time::now() - RosCanConstants::max_stm_sync_time)
	{
		sWheelsVel.header.stamp = ros::Time::now();
		ROS_WARN_COND(!sWasMotorVelPublishedSinceWheelsVelStampChange, "CAN: \"Wheels velocity feedback\" sync time exceeded. Frame dropped.");
		sWasMotorVelPublishedSinceWheelsVelStampChange = false;
		return;
	}
	sWheelsVel.header.seq++;
	sWasMotorVelPublishedSinceWheelsVelStampChange = true;
	sRealMotorVelPub.publish(sWheelsVel);
	ros::spinOnce();
}