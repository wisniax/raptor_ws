#include "can_wrapper/MotorCurrentFeedback.hpp"

MotorCurrentFeedback::MotorCurrentFeedback(float curr_scale)
{
	mCurr_scale = curr_scale;
	mRawCanSub = mNh.subscribe(RosCanConstants::RosTopics::can_raw_RX, 256, &MotorCurrentFeedback::handleRosCallback, this);
	mCurrFeedbackRequestPub = mNh.advertise<can_msgs::Frame>(RosCanConstants::RosTopics::can_raw_TX, 128);
	mRealMotorCurrPub = mNh.advertise<can_wrapper::Wheels>(RosCanConstants::RosTopics::can_get_motor_current, 128);
}

void MotorCurrentFeedback::handleRequestTimerCallback(const ros::TimerEvent &)
{
	sendRequest();
}

void MotorCurrentFeedback::sendRequest()
{
	CM_CanMessage cm;
	cm.address = CM_ADDRESS_STM_RIGHT | CM_ADDRESS_FAMILY_MOTOR_CURRENT_FEEDBACK | CAN_RTR_FLAG;
	cm.dataLen = 0;
	mCurrFeedbackRequestPub.publish(CM_convert_CanMessage_mtof(cm));
}

void MotorCurrentFeedback::handleRosCallback(const can_msgs::Frame::ConstPtr &msg)
{
	if ((msg->id & CM_ADDRESS_FAMILY_MASK) != CM_ADDRESS_FAMILY_MOTOR_CURRENT_FEEDBACK)
		return;
	handleFrame(CM_convert_CanMessage_ftom(msg.get()));
}

void MotorCurrentFeedback::handleFrame(CM_CanMessage cm)
{
	CM_Vector3f vec;
	switch (cm.address & CM_ADDRESS_MASK)
	{
	case CM_ADDRESS_STM_LEFT:
		vec = decodeMotorCurr(cm);
		mWheelsCurr.frontLeft = vec.x;
		mWheelsCurr.midLeft = vec.y;
		mWheelsCurr.rearLeft = vec.z;
		tryPublishWheelsCurr();
		break;
	case CM_ADDRESS_STM_RIGHT:
		vec = decodeMotorCurr(cm);
		mWheelsCurr.frontRight = vec.x;
		mWheelsCurr.midRight = vec.y;
		mWheelsCurr.rearRight = vec.z;
		tryPublishWheelsCurr();
		break;
	case CM_ADDRESS_STM_ARM_AXIS_123: // Work in progress
		break;
	case CM_ADDRESS_STM_ARM_AXIS_456: // Work in progress
		break;
	default:
		break;
	}
}

CM_Vector3f MotorCurrentFeedback::decodeMotorCurr(CM_CanMessage cm) const
{
	float curr_scale[3] = {mCurr_scale, mCurr_scale, mCurr_scale};

	CM_Vector3f vec;
	switch (cm.data.mode.cont_mode)
	{
	case CM_GETMOTORVEL_CONTMODE_RPM:
		break;
	case CM_GETMOTORVEL_CONTMODE_RPM_NEWSCALE:
		break;
	default:
		break;
	}
	return vec;
}

void MotorCurrentFeedback::tryPublishWheelsCurr()
{
	if (mWheelsCurr.header.stamp < ros::Time::now() - RosCanConstants::max_stm_sync_time) // TODO: Sync should not take the same value from one side twice.
	{
		mWheelsCurr.header.stamp = ros::Time::now();
		ROS_WARN_COND(!mWasMotorCurrPublishedSinceStampChange, "CAN: \"Wheels current feedback\" sync time exceeded. Frame dropped.");
		mWasMotorCurrPublishedSinceStampChange = false;
		return;
	}
	mWheelsCurr.header.seq++;
	mWasMotorCurrPublishedSinceStampChange = true;
	mRealMotorCurrPub.publish(mWheelsCurr);
	ros::spinOnce();
}