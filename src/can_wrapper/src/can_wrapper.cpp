
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <memory>
#include <linux/can.h>
#include "can_wrapper/CanMessage.hpp"

#define alignTest(x) ROS_INFO(#x" address: 0x%p sizeof: %u align: %u", (void*)&x , (uint32_t)sizeof(x), (uint32_t)alignof(x))

void chatterCallback(const std_msgs::String::ConstPtr& msg)
{

}

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "can_wrapper");

	ros::NodeHandle n;
	ros::Publisher chatter_pub = n.advertise<std_msgs::String>("/CAN/RX/raw", 256);
	ros::Subscriber sub = n.subscribe("/CAN/TX/raw", 256, chatterCallback);

	ros::Rate loop_rate(1);
	

	while (ros::ok)
	{
		ROS_WARN("JD");
		ros::spinOnce();
		loop_rate.sleep();
	}
}