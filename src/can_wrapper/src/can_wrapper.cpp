// Last modified by: Marcin Walczyk/12.10.2023

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <memory>
#include <linux/can.h>
#include "can_wrapper/CanMessage.hpp"

#define alignTest(x) ROS_INFO(#x" address: 0x%p sizeof: %u align: %u", (void*)&x , (uint32_t)sizeof(x), (uint32_t)alignof(x))

// void rx_er(boost::shared_ptr<const int32_t> smth)
// {
//     ROS_INFO("Recived: %i", smth.get());
// }
int main(int argc, char *argv[])
{
	ros::init(argc, argv, "can_wrapper");
	
	//CallbackCanMessage ccwm(CanMessage::Address::TX_ArmAxis123, 132ui16, -1i8);

	ros::NodeHandle n;
	ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
   // ros::Subscriber sub = n.subscribe<int32_t>("lmao", 1000, rx_er);

	ros::Rate loop_rate(1);

	cm::CanMessage cm;

	memset(&cm,0,sizeof(cm));

	alignTest(cm);
	alignTest(cm.address);
	alignTest(cm.dataLength);
	alignTest(cm.data);
	alignTest(cm.data.raw);

	while (ros::ok)
	{
		ROS_INFO("JD");
		ros::spinOnce();
		loop_rate.sleep();
	}
}