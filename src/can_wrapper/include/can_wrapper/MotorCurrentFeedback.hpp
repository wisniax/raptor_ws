#ifndef MOTOR_CURRENT_FEEDBACK_H
#define MOTOR_CURRENT_FEEDBACK_H

#include <ros/ros.h>
#include <string>
#include <can_msgs/Frame.h>
#include <geometry_msgs/Point32.h>
#include <memory>
#include <linux/can.h>
#include "can_wrapper/Wheels.h"
#include "can_wrapper/RosCanConstants.hpp"
#include "CM/CM.h"

class MotorCurrentFeedback
{
public:
	/**
	 * @brief Initializes the MotorCurrentFeedback object.
	 * @param rpm_scale The scale factor for motor RPM.
	 */
	MotorCurrentFeedback(float curr_scale);
	void handleRequestTimerCallback(const ros::TimerEvent &);
	void sendRequest();

private:
	void handleFrame(CM_CanMessage cm);
	CM_Vector3f decodeMotorCurr(CM_CanMessage cm) const;
	void tryPublishWheelsCurr();
	void handleRosCallback(const can_msgs::Frame::ConstPtr &msg);

private:
	ros::NodeHandle mNh;
	float mCurr_scale;
	can_wrapper::Wheels mWheelsCurr;
	bool mWasMotorCurrPublishedSinceStampChange;

	ros::Subscriber mRawCanSub;
	ros::Publisher mRealMotorCurrPub;
	ros::Publisher mCurrFeedbackRequestPub;
};

#endif // MOTOR_CURRENT_FEEDBACK_H