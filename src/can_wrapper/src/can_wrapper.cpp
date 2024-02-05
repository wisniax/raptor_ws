#include <ros/ros.h>
#include <memory>
#include <linux/can.h>
#include <can_msgs/Frame.h>
#include <std_msgs/String.h>

#include "CM/CM.h"
#include "can_wrapper/CanNodeSettingsProvider.hpp"
#include "can_wrapper/MotorVelocityFeedback.hpp"
#include "can_wrapper/MotorControl.hpp"
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

	MotorVelocityFeedback velFeedback(
		canSettingsPtr->getSetting(
			CM_ADDRESS_STM_LEFT,
			CM_STMINIT_TYPEID_FAMILY_MOTORCONTROL | CM_STMINIT_TYPEID_MOTORCONTROL_FEEDBACK));

	MotorControl motorControl(
		canSettingsPtr->getSetting(
			CM_ADDRESS_STM_LEFT,
			CM_STMINIT_TYPEID_FAMILY_MOTORCONTROL | CM_STMINIT_TYPEID_MOTORCONTROL_COMMAND),
		CM_SETMOTORVEL_CONTMODE_PWM);

	CanNodeErrorHandler canErrorHandler(canSettingsPtr);

	CanNodeMode canNodeMode = CanNodeMode::Created;
	ros::Rate rate(1000);

	ros::Timer timer = n.createTimer(ros::Duration(1.0 / 60.0), &MotorVelocityFeedback::handleRequestTimerCallback, &velFeedback, false, false);

	std_srvs::SetBool::Request req;
	std_srvs::SetBool::Response res;
	can_wrapper::Wheels vel;

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

			if (res.success != 0)
			{
				ROS_WARN("Package ros_can_integration is not yet ready... Retrying...");
				canNodeMode = CanNodeMode::Faulted;
				break;
			}
			canErrorHandler.requestDeinitialization();
			canNodeMode = CanNodeMode::Opening;
			break;

		case CanNodeMode::Opening:
			canErrorHandler.initializeDevices();
			ros::Duration(0.1).sleep();
			
			if (canErrorHandler.GetCanNodesStatus())
				canNodeMode = CanNodeMode::Opened;

			break;

		case CanNodeMode::Opened:
			if (!timer.hasStarted())
				timer.start();
			break;

		case CanNodeMode::Closing:
			ROS_INFO("CanNodeMode::Closing");
			timer.stop();

			canNodeMode = CanNodeMode::Closed;
			break;
		case CanNodeMode::Closed:

			canNodeMode = CanNodeMode::Opening;
			break;
		case CanNodeMode::Faulted:
			timer.stop();
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