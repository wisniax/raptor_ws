#include <ros/ros.h>
#include <memory>
#include <linux/can.h>
#include <can_msgs/Frame.h>
#include <std_msgs/String.h>

#include "can_wrapper/CanNodeSettingsProvider.hpp"
#include "can_wrapper/CanMessage.hpp"
#include "can_wrapper/Can2Ros.hpp"
#include "can_wrapper/Ros2Can.hpp"
#include "can_wrapper/RosCanConstants.hpp"
#include "can_wrapper/CanNodeErrorHandler.hpp"

#include <ros/service.h>
#include <std_srvs/SetBool.h>

enum class CanNodeMode
{
	Created,
	Opening,
	Opened,
	Closing,
	Closed,
	Faulted
};

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "can_wrapper");
	ros::NodeHandle n;

	std::shared_ptr<CanNodeSettingsProvider> canSettingsPtr = std::make_shared<CanNodeSettingsProvider>();

	Can2Ros can2ros(
		canSettingsPtr->getSetting(
			0x0,
			CanNodeSettingsProvider::RpmScaleAdresses::Encoder_Feedback));

	Ros2Can ros2can(
		canSettingsPtr->getSetting(
			0x0,
			CanNodeSettingsProvider::RpmScaleAdresses::Motor_Control),
		CanMessage::set_motor_vel_t::mode_cont_mode::TargetModePwm);

	CanNodeErrorHandler canErrorHandler(canSettingsPtr);

	CanNodeMode canNodeMode = CanNodeMode::Created;
	ros::Rate rate(1000);

	std_srvs::SetBool::Request req;
	std_srvs::SetBool::Response res;
	float vel[3] = {0};

	while (ros::ok())
	{
		switch (canNodeMode)
		{
		case CanNodeMode::Created:

			if (!ros::service::waitForService("/CAN/ros_can_integration/check_status", 15))
			{
				ROS_WARN("Service /CAN/ros_can_integration/check_status is not yet available. Retrying...");
				canNodeMode = CanNodeMode::Faulted;
				break;
			}

			if (!ros::service::call("/CAN/ros_can_integration/check_status", req, res))
			{
				ROS_WARN("Service /CAN/ros_can_integration/check_status call failed. Retrying...");
				canNodeMode = CanNodeMode::Faulted;
				break;
			}

			if (!res.success)
			{
				ROS_WARN("Package ros_can_integration is not yet ready... Retrying...");
				canNodeMode = CanNodeMode::Faulted;
				break;
			}

			canNodeMode = CanNodeMode::Opening;
			break;

		case CanNodeMode::Opening:
			canErrorHandler.initializeDevices();
			ros::Duration(0.1).sleep();
			if (canErrorHandler.GetCanNodesStatus())
				canNodeMode = CanNodeMode::Opened;
			break;

		case CanNodeMode::Opened:
			vel[0] = 0.5f;
			ros::Duration(0.1).sleep();
			ros2can.setMotorVel(vel, CanMessage::Address::Stm_Right);
			break;

		case CanNodeMode::Closing:
			ROS_INFO("CanNodeMode::Closing");

			canNodeMode = CanNodeMode::Closed;
			break;
		case CanNodeMode::Closed:

			canNodeMode = CanNodeMode::Opening;
			break;
		case CanNodeMode::Faulted:
			ros::Duration(5).sleep();
			canNodeMode = CanNodeMode::Created;
			break;
		default:
			ROS_INFO("CanNodeMode::Unknown");
			canNodeMode = CanNodeMode::Faulted;
			break;
		}
		rate.sleep();
		ros::spinOnce();
	}
}