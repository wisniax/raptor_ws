#include <ros/ros.h>
#include <memory>
#include <linux/can.h>
#include <can_msgs/Frame.h>
#include <std_msgs/String.h>

#include "can_wrapper/CanMessage.hpp"
#include "can_wrapper/CanSocket.hpp"
#include "can_wrapper/Can2Ros.hpp"

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "can_wrapper");
	ros::NodeHandle n;

	static CanSocket cSocket("can0");

	ros::Publisher canRawPub = n.advertise<can_msgs::Frame>("/CAN/RX/raw", 256);
	ros::Subscriber canRawSub = n.subscribe(
		"/CAN/TX/raw",
		256,
		((void (*)(const can_msgs::Frame::ConstPtr &))[](const can_msgs::Frame::ConstPtr &msg) {
			cSocket.handleRosCallback(msg);
		}));

	ros::Rate loop_rate(1);
	std_msgs::String canErrStr;
	canErrStr.data = "CAN: " + cSocket.translateInitError();
	ROS_INFO_STREAM(canErrStr);

	while (ros::ok)
	{
		// can_msgs::Frame msg;
		// msg.data = {6,9};
		// canRawPub.publish(msg);
		CanMessage cm;
		cSocket.awaitMessage(cm);




		ros::spinOnce();
		//loop_rate.sleep();
	}
}