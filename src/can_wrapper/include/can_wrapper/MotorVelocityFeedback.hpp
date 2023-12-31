#ifndef MOTOR_VELOCITY_FEEDBACK_H
#define MOTOR_VELOCITY_FEEDBACK_H

#include <ros/ros.h>
#include <string>
#include <can_msgs/Frame.h>
#include <geometry_msgs/Point32.h>
#include <memory>
#include <linux/can.h>
#include "can_wrapper/Wheels.h"
#include "can_wrapper/RosCanConstants.hpp"
#include "CM/CM.h"

class MotorVelocityFeedback
{
public:
	/**
	 * @brief Initializes the MotorVelocityFeedback object.
	 * @param rpm_scale The scale factor for motor RPM.
	 */
	MotorVelocityFeedback(float rpm_scale);
	void handleRequestTimerCallback(const ros::TimerEvent &);
	void sendRequest();

private:
	void handleFrame(CM_CanMessage cm);
	CM_Vector3f decodeMotorVel(CM_CanMessage cm) const;
	void tryPublishWheelsVel();
	void handleRosCallback(const can_msgs::Frame::ConstPtr &msg);

private:
	ros::NodeHandle mNh;
	float mRPM_scale;
	can_wrapper::Wheels mWheelsVel;
	bool mWasMotorVelPublishedSinceWheelsVelStampChange;

	ros::Subscriber mRawCanSub;
	ros::Publisher mRealMotorVelPub;
	ros::Publisher mFeedbackRequestPub;
};

#endif // MOTOR_VELOCITY_FEEDBACK_H