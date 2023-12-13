#include <ros/ros.h>
#include <memory>
#include <linux/can.h>
#include <can_msgs/Frame.h>
#include <std_msgs/String.h>
#include <std_srvs/SetBool.h>

#include "ros_can_integration/CanSocket.hpp"

CanSocket cSocket("can0");

bool getErrorCodeCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{
	res.success = cSocket.getErrorCode() == 0 ? true : false;
	if (!res.success)
		res.message = cSocket.translateInitError();
	return true;
};

void tryGetCanFrame(uint32_t& seq, ros::Publisher canRawPub)
{
	can_frame frame;
		if (cSocket.awaitMessage(frame) < 0)
		{
			if (cSocket.tryHandleError() != 0)
				return;

			ros::Duration(0.0005).sleep();
			return;
		}

		can_msgs::Frame fr;
		fr.id = frame.can_id;
		fr.dlc = frame.can_dlc;
		memcpy(fr.data.data(), frame.data, CAN_MAX_DLEN);
		fr.header.stamp = ros::Time::now();
		fr.header.seq = seq++;
		canRawPub.publish(fr);
}

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "ros_can_integration");
	ros::NodeHandle n;

	ros::Publisher canRawPub = n.advertise<can_msgs::Frame>("/CAN/RX/raw", 256);
	ros::Subscriber canRawSub = n.subscribe("/CAN/TX/raw", 256, &CanSocket::handleRosCallback, &cSocket);

	ros::ServiceServer service = n.advertiseService("/CAN/ros_can_integration/check_status", getErrorCodeCallback);

	uint32_t seq = 0;
	ros::Rate rate(1000);

	while (ros::ok)
	{
		tryGetCanFrame(seq, canRawPub);
		
		rate.sleep();
		ros::spinOnce();
	}
}