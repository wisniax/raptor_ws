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

static CanSocket cSocket("can0");
ros::Publisher canRawPub;

void rosSpinLmao(const ros::TimerEvent &)
{
	CanNodeErrorHandler::initializeDevices();
	uint32_t seq = 0;

	CanMessage cm;
	if (cSocket.awaitMessage(cm) < 0)
	{
		if (cSocket.tryHandleError() != 0)
			return;

		ros::Duration(0.0005).sleep();
		return;
	}
	can_msgs::Frame fr = (can_msgs::Frame)cm;
	fr.header.seq = seq++;
	canRawPub.publish(fr);
}

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "can_wrapper");
	ros::NodeHandle n;

	canRawPub = n.advertise<can_msgs::Frame>(RosCanConstants::RosTopics::can_raw_RX, 256);
	ros::Subscriber canRawSub = n.subscribe(
		RosCanConstants::RosTopics::can_raw_TX,
		256,
		((void (*)(const can_msgs::Frame::ConstPtr &))[](const can_msgs::Frame::ConstPtr &msg) {
			cSocket.handleRosCallback(msg);
		}));

	CanNodeSettingsProvider::init();

	Can2Ros::init(CanNodeSettingsProvider::getSetting(0x0, CanNodeSettingsProvider::RpmScaleAdresses::Encoder_Feedback), n);

	Ros2Can::init(CanNodeSettingsProvider::getSetting(0x0, CanNodeSettingsProvider::RpmScaleAdresses::Motor_Control), n,
				  CanMessage::set_motor_vel_t::mode_cont_mode::TargetModePwm);

	CanNodeErrorHandler::init(n);

	// ros::Duration(1).sleep();

	bool isStmInitialized = false;
	ros::Timer timer = n.createTimer(ros::Duration(0.001), rosSpinLmao);
	timer.start();
	ros::spin();
	// while (ros::ok)
	// {
	// }
}