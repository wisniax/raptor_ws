#include "can_wrapper/MotorVelocityFeedback.hpp"

MotorVelocityFeedback::MotorVelocityFeedback(float rpm_scale)
{
	mRPM_scale = rpm_scale;
	mRawCanSub = mNh.subscribe(RosCanConstants::RosTopics::can_raw_RX, 256, &MotorVelocityFeedback::handleRosCallback, this);
	mFeedbackRequestPub = mNh.advertise<can_msgs::Frame>(RosCanConstants::RosTopics::can_raw_TX, 128);
	mRealMotorVelPub = mNh.advertise<can_wrapper::Wheels>(RosCanConstants::RosTopics::can_get_motor_vel, 128);
}

void MotorVelocityFeedback::handleRequestTimerCallback(const ros::TimerEvent &)
{
	sendRequest();
}

void MotorVelocityFeedback::sendRequest()
{
	CM_CanMessage cm;
	cm.address = CM_ADDRESS_STM_RIGHT | CM_ADDRESS_FAMILY_ENCODER_VELOCITY_FEEDBACK | CAN_RTR_FLAG;
	cm.dataLen = 0;
	mFeedbackRequestPub.publish(CM_convert_CanMessage_mtof(cm));
}

void MotorVelocityFeedback::handleRosCallback(const can_msgs::Frame::ConstPtr &msg)
{
	if ((msg->id & CM_ADDRESS_FAMILY_MASK) != CM_ADDRESS_FAMILY_ENCODER_VELOCITY_FEEDBACK)
		return;
	handleFrame(CM_convert_CanMessage_ftom(msg.get()));
}

void MotorVelocityFeedback::handleFrame(CM_CanMessage cm)
{
	CM_Vector3f vec;
	switch (cm.address & CM_ADDRESS_MASK)
	{
	case CM_ADDRESS_STM_LEFT:
		vec = decodeMotorVel(cm);
		mWheelsVel.frontLeft = vec.x;
		mWheelsVel.midLeft = vec.y;
		mWheelsVel.rearLeft = vec.z;
		tryPublishWheelsVel();
		break;
	case CM_ADDRESS_STM_RIGHT:
		vec = decodeMotorVel(cm);
		mWheelsVel.frontRight = vec.x;
		mWheelsVel.midRight = vec.y;
		mWheelsVel.rearRight = vec.z;
		tryPublishWheelsVel();
		break;
	case CM_ADDRESS_STM_ARM_AXIS_123: // Work in progress
		break;
	case CM_ADDRESS_STM_ARM_AXIS_456: // Work in progress
		break;
	default:
		break;
	}
}

CM_Vector3f MotorVelocityFeedback::decodeMotorVel(CM_CanMessage cm) const
{
	float rpm_scale[3] = {mRPM_scale,mRPM_scale,mRPM_scale};

	CM_Vector3f vec;
	switch (cm.data.mode.cont_mode)
	{
	case CM_GETMOTORVEL_CONTMODE_RPM:
		CM_convert_GetMotorVel_mtor_M(&vec,rpm_scale,&cm.data.get_motor_vel);
		break;
	case CM_GETMOTORVEL_CONTMODE_RPM_NEWSCALE:
		CM_convert_GetMotorVel_mtor_NM(&vec,rpm_scale,&cm.data.get_motor_vel);
		break;
	default:
		break;
	}
	return vec;
}

void MotorVelocityFeedback::tryPublishWheelsVel()
{
	if (mWheelsVel.header.stamp < ros::Time::now() - RosCanConstants::max_stm_sync_time) // TODO: Sync should not take the same value from one side twice.
	{
		mWheelsVel.header.stamp = ros::Time::now();
		ROS_WARN_COND(!mWasMotorVelPublishedSinceWheelsVelStampChange, "CAN: \"Wheels velocity feedback\" sync time exceeded. Frame dropped.");
		mWasMotorVelPublishedSinceWheelsVelStampChange = false;
		return;
	}
	mWheelsVel.header.seq++;
	mWasMotorVelPublishedSinceWheelsVelStampChange = true;
	mRealMotorVelPub.publish(mWheelsVel);
	ros::spinOnce();
}