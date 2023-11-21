#ifndef CAN2ROS_H
#define CAN2ROS_H

#include <ros/ros.h>
#include <string>
#include <can_msgs/Frame.h>
#include <geometry_msgs/Point32.h>
#include <memory>
#include "can_wrapper/CanMessage.hpp"
#include "can_wrapper/Wheels.h"
#include "can_wrapper/RosCanConstants.hpp"

class Can2Ros
{
public:
	Can2Ros(float rpm_scale);

private:
	void handleFrame(CanMessage cm);
	geometry_msgs::Point32 decodeMotorVel(CanMessage cm) const;
	void tryPublishWheelsVel();
	void handleRosCallback(const can_msgs::Frame::ConstPtr &msg);

private:
	ros::NodeHandle mNh;
	float mRPM_scale;
	can_wrapper::Wheels mWheelsVel;
	bool mWasMotorVelPublishedSinceWheelsVelStampChange;

	ros::Subscriber mRawCanSub;
	ros::Publisher mRealMotorVelPub;
};

#endif // CAN2ROS_H