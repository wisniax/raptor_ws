
#include <ros/ros.h>
#include <memory>
#include <linux/can.h>

#include "can_wrapper/CanMessage.hpp"
#include "can_wrapper/CanSocket.hpp"

// Example of Ros Callback. to be deleted
// void transferCanCallback(const std_msgs::String::ConstPtr& msg)
// {
	
// }

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "can_wrapper");
	ros::NodeHandle n;
	CanSocket cSocket("can0");
	std::string errStr = cSocket.translateInitError();
	//ROS_DEBUG(errStr);

	ros::Publisher chatter_pub = n.advertise<std_msgs::String>("/CAN/RX/raw", 256);
	ros::Subscriber sub = n.subscribe("/CAN/TX/raw", 256, cSocket.handleRosCallback);

	ros::Rate loop_rate(1);
	

	while (ros::ok)
	{
		ROS_WARN("JD");
		chatter_pub.publish("HAH");
		ros::spinOnce();
		loop_rate.sleep();
	}
}