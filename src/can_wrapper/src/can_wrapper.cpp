#include <ros/ros.h>
#include <memory>
#include <linux/can.h>
#include <can_msgs/Frame.h>
#include <std_msgs/String.h>

#include "can_wrapper/CanNodeSettingsProvider.hpp"
#include "can_wrapper/CanMessage.hpp"
#include "can_wrapper/CanSocket.hpp"
#include "can_wrapper/Can2Ros.hpp"
#include "can_wrapper/Ros2Can.hpp"
#include "can_wrapper/RosCanConstants.hpp"
#include "can_wrapper/CanNodeErrorHandler.hpp"

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "can_wrapper");
	ros::NodeHandle n;

	static CanSocket cSocket("can0");

	ros::Publisher canRawPub = n.advertise<can_msgs::Frame>(RosCanConstants::RosTopics::can_raw_RX, 256);
	ros::Subscriber canRawSub = n.subscribe(
		RosCanConstants::RosTopics::can_raw_TX,
		256,
		((void (*)(const can_msgs::Frame::ConstPtr &))[](const can_msgs::Frame::ConstPtr &msg) {
			cSocket.handleRosCallback(msg);
		}));

	CanNodeSettingsProvider::init();

	Can2Ros::init(CanNodeSettingsProvider::getSetting(0x0, CanNodeSettingsProvider::RpmScaleAdresses::Encoder_Feedback));

	Ros2Can::init(CanNodeSettingsProvider::getSetting(0x0, CanNodeSettingsProvider::RpmScaleAdresses::Motor_Control),
				  CanMessage::set_motor_vel_t::mode_cont_mode::TargetModePwm);

	CanNodeErrorHandler::init();

	uint32_t seq = 0;
	while (ros::ok)
	{
		CanMessage cm;
		if (cSocket.awaitMessage(cm) < 0)
		{
			if (cSocket.tryHandleError() != 0)
				continue;

			ros::Duration(0.0005).sleep();
			continue;
		}
		can_msgs::Frame fr = (can_msgs::Frame)cm;
		fr.header.seq = seq++;
		canRawPub.publish(fr);
		ros::spinOnce();
	}
}