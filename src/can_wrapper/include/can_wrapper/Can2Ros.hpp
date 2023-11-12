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
	ros::NodeHandle nh;
	float mRPM_scale;
	static std::unique_ptr<Can2Ros> instance;
	can_wrapper::Wheels mWheelsVel;
	bool mWasMotorVelPublishedSinceWheelsVelStampChange;

	ros::Subscriber mRawCanSub;
	ros::Publisher mRealMotorVelPub;

	void handleFrame(CanMessage cm);
	geometry_msgs::Point32 decodeMotorVel(CanMessage cm);
	void tryPublishWheelsVel();
	static void handleRosCallback(const can_msgs::Frame::ConstPtr &msg);

public:
	Can2Ros() = default;
	Can2Ros(Can2Ros &) = delete;
	void operator=(const Can2Ros &) = delete;

	static Can2Ros *getSingleton();
	void init(float rpm_scale);

};

#endif // CAN2ROS_H