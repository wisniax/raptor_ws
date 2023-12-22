#include <ros/ros.h>
#include <memory>
#include <linux/can.h>
#include <can_msgs/Frame.h>
#include <std_msgs/Int32.h>

#include "ros_can_integration/CanSocket.hpp"

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "ros_can_integration");
	ros::NodeHandle n;
	CanSocket cSocket("can0");

	ros::Publisher canRawPub = n.advertise<can_msgs::Frame>("/CAN/RX/raw", 1024);
	ros::Subscriber canRawSub = n.subscribe("/CAN/TX/raw", 1024, &CanSocket::handleRosCallback, &cSocket);

	ros::ServiceServer service = n.advertiseService("/CAN/ros_can_integration/check_status", &CanSocket::getErrorCodeCallback, &cSocket);
	ros::Publisher canStatusPub = n.advertise<std_msgs::Int32>("/CAN/ros_can_integration/status", 16, true);
	int lastErrCode = 2;
	ros::Rate rate(1000);

	while (ros::ok)
	{
		if (lastErrCode != cSocket.getErrorCode())
		{
			lastErrCode = cSocket.getErrorCode();
			std_msgs::Int32 msg;
			msg.data = lastErrCode;
			canStatusPub.publish(msg);
		}
		cSocket.awaitAndPublishCanMessage(canRawPub);

		rate.sleep();
		ros::spinOnce();
	}
}