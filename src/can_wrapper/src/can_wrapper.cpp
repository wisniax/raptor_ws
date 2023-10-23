
#include <ros/ros.h>
#include <memory>
#include <linux/can.h>

#include "can_wrapper/CanMessage.hpp"
#include "can_wrapper/CanSocket.hpp"

// Example of Ros Callback. to be deleted



int main(int argc, char *argv[])
{
	ros::init(argc, argv, "can_wrapper");
	ros::NodeHandle n;

	static CanSocket cSocket("can0");

	std::string errStr = cSocket.translateInitError();
	//ROS_DEBUG(errStr);

	ros::Publisher chatter_pub = n.advertise<std_msgs::String>("/CAN/RX/raw", 256);
	ros::Subscriber sub = n.subscribe
	(
		"/CAN/TX/raw",
		256,
		((void(*)(const std_msgs::String::ConstPtr&))[] (const std_msgs::String::ConstPtr& msg)
		{ 
			cSocket.handleRosCallback(msg);
		})
	);

	ros::Rate loop_rate(1);
	

	while (ros::ok)
	{
		ROS_WARN("JD");
		std_msgs::String yyy;
		yyy.data = "HAH";
		chatter_pub.publish(yyy);
		ros::spinOnce();
		loop_rate.sleep();
	}
}