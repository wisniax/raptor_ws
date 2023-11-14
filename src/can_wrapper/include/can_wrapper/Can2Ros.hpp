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
private:
	static bool sIsInitialized;
	static ros::NodeHandle sNh;
	static float sRPM_scale;
	static can_wrapper::Wheels sWheelsVel;
	static bool sWasMotorVelPublishedSinceWheelsVelStampChange;

	static ros::Subscriber sRawCanSub;
	static ros::Publisher sRealMotorVelPub;

	static void handleFrame(CanMessage cm);
	static geometry_msgs::Point32 decodeMotorVel(CanMessage cm);
	static void tryPublishWheelsVel();
	static void handleRosCallback(const can_msgs::Frame::ConstPtr &msg);

public:
	Can2Ros() = delete;

	static void init(float rpm_scale);
};

#endif // CAN2ROS_H