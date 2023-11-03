#ifndef ROS2CAN_H
#define ROS2CAN_H

#include <ros/ros.h>
#include <string>
#include <can_msgs/Frame.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <memory>
#include "can_wrapper/CanMessage.hpp"

class Ros2Can
{
private:
	ros::NodeHandle nh;
	float mRPM_scale;
	static std::unique_ptr<Ros2Can> instance;
	void handleFrame(CanMessage cm);
	can_msgs::Frame encodeMotorVel(geometry_msgs::Vector3Stamped msg, CanMessage::Address adr);

	ros::Publisher mRawCanPub;
	ros::Subscriber mDriversLeft;
	ros::Subscriber mDriversRight;
	ros::Subscriber mArm123;
	ros::Subscriber mArm456;

public:
	Ros2Can() = default;
	Ros2Can(Ros2Can &) = delete;
	void operator=(const Ros2Can &) = delete;

	static Ros2Can *getSingleton();
	void init(std::string can_RX_topic = "/CAN/TX/", float rpm_scale = 1);

	static void handleSetLeftDriverVel(const geometry_msgs::Vector3Stamped &msg);
	static void handleSetRightDriverVel(const geometry_msgs::Vector3Stamped &msg);
	static void handleSetArm123Vel(const geometry_msgs::Vector3Stamped &msg);
	static void handleSetArm456Vel(const geometry_msgs::Vector3Stamped &msg);
};

#endif // ROS2CAN_H