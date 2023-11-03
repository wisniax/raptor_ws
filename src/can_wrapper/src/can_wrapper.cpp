#include <ros/ros.h>
#include <memory>
#include <linux/can.h>
#include <can_msgs/Frame.h>
#include <std_msgs/String.h>
#include <iostream>
#include <fstream>

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

	CanMessage arr[100];
	uint64_t i = 0;
	int32_t j = 0;
	uint32_t errCnt = 0;
	std::ofstream MyFile("/home/raptors/CanTests/raptor_ws/canErrors.csv");
	MyFile << "time,id,number" << std::endl;

	while (ros::ok)
	{
		// can_msgs::Frame msg;
		// msg.data = {6,9};
		// canRawPub.publish(msg);
		CanMessage cm;
		cSocket.awaitMessage(cm);

		if ((cm.address & CAN_ERR_FLAG) > 0) 
		{
			errCnt++;
			continue;
		}

		uint64_t erMarche;
		std::memcpy(&erMarche, cm.data.raw, sizeof(erMarche));

		if (i == 0)
			i = erMarche;
		else if ((i + 1) == erMarche)
			i = erMarche;
		else
		{
			arr[j++] = cm;
			i = 0;
			if (j >= 30)
			{
				for (int x = 30 - 1; x >= 0; x--)
					MyFile << ros::Time::now().toSec() << ',' << arr[x].address << ',' << *reinterpret_cast<uint64_t*>(arr[x].data.raw) << std::endl;
				// std::cout << arr[x] << std::endl;
				throw("Out Of Range");
			}
		}

		ros::spinOnce();
		// loop_rate.sleep();
	}
}