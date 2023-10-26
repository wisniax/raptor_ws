
#include <ros/ros.h>
#include <memory>
#include <linux/can.h>

#include "can_wrapper/CanMessage.hpp"
#include "can_wrapper/CanSocket.hpp"
#include "can_wrapper/raw_can_message.h"

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "can_wrapper");
	ros::NodeHandle n;

	static CanSocket cSocket("can0");

	ros::Publisher chatter_pub = n.advertise<std_msgs::String>("/CAN/RX/raw", 256);
	ros::Subscriber sub = n.subscribe(
		"/CAN/TX/raw",
		256,
		((void (*)(const std_msgs::String::ConstPtr &))[](const std_msgs::String::ConstPtr &msg) {
			cSocket.handleRosCallback(msg);
		}));

	ros::Rate loop_rate(1);
	std_msgs::String canErrStr;
	canErrStr.data = "CAN: " + cSocket.translateInitError();
	ROS_INFO_STREAM(canErrStr);

	while (ros::ok)
	{
		std_msgs::String msg;
		msg.data = "HAH";
		chatter_pub.publish(msg);

		ros::spinOnce();
		loop_rate.sleep();
	}
}