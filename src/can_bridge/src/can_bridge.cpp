#include "rclcpp/rclcpp.hpp"
#include <memory>
#include <linux/can.h>

#include "can_msgs/msg/frame.hpp"
#include "std_msgs/msg/string.hpp"

#include "std_srvs/srv/set_bool.hpp"

// #include "can_wrapper/MotorControl.hpp"
// #include "can_wrapper/RosCanConstants.hpp"
// #include "can_wrapper/VescStatusHandler.hpp"
// #include "can_wrapper/RoverControl.h"
// #include "can_wrapper/StatusMessage.hpp"
// #include "can_wrapper/ManipulatorControl.hpp"
// #include "can_wrapper/ProbeStatusForwarder.hpp"

// #include <std_srvs/SetBool.h>

enum class CanNodeMode
{
	Created,
	Opening,
	Opened,
	Closing,
	Closed,
	Faulted
};

// void doDrivingStuff(MotorControl &mtrCtl);

static std::chrono::system_clock::time_point lastSendWheels;
static std::chrono::nanoseconds diff;

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	auto n = rclcpp::Node::make_shared("can_bridge");

	// if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Warn))
	// 	ros::console::notifyLoggerLevelsChanged();

	// MotorControl motorControl(n);
	// VescStatusHandler mVescStatusHandler(n);
	// StatusMessage mStatusMessage(n, true);
	// ManipulatorControl mManipulatorCtl(n);
	// ProbeStatusForwarder mProbeStatusForwarder(n);

	CanNodeMode canNodeMode = CanNodeMode::Created;
	rclcpp::Rate rate(100);

	// std_srvs::SetBool::Request req;
	// std_srvs::SetBool::Response res;

	int iter = 0;

	while (rclcpp::ok())
	{
		// can_wrapper::Wheels vel;
		// switch (canNodeMode)
		// {
		// case CanNodeMode::Created:

		// 	if (!ros::service::waitForService("/CAN/ros_can_integration/check_status", 15))
		// 	{
		// 		ROS_WARN("Service /CAN/ros_can_integration/check_status is not yet available. Retrying...");
		// 		canNodeMode = CanNodeMode::Faulted;
		// 		break;
		// 	}

		// 	if (!ros::service::call("/CAN/ros_can_integration/check_status", req, res))
		// 	{
		// 		ROS_WARN("Service /CAN/ros_can_integration/check_status call failed. Retrying...");
		// 		canNodeMode = CanNodeMode::Faulted;
		// 		break;
		// 	}

		// 	if (res.success != 0)
		// 	{
		// 		ROS_WARN("Package ros_can_integration is not yet ready... Retrying...");
		// 		canNodeMode = CanNodeMode::Faulted;
		// 		break;
		// 	}
		// 	canNodeMode = CanNodeMode::Opening;
		// 	break;

		// case CanNodeMode::Opening:
		// 	ros::Duration(0.1).sleep();

		// 	canNodeMode = CanNodeMode::Opened;

		// 	break;

		// case CanNodeMode::Opened:
		// 	if ((iter++ % 100) == 9)
		// 	{
		// 		mStatusMessage.sendStatusMessage();
		// 	}

		// 	break;

		// case CanNodeMode::Closing:
		// 	ROS_INFO("CanNodeMode::Closing");

		// 	canNodeMode = CanNodeMode::Closed;
		// 	break;
		// case CanNodeMode::Closed:

		// 	canNodeMode = CanNodeMode::Opening;
		// 	break;
		// case CanNodeMode::Faulted:
		// 	ros::Duration(5).sleep();
		// 	canNodeMode = CanNodeMode::Created;
		// 	break;
		// default:
		// 	ROS_INFO("CanNodeMode::Unknown");
		// 	canNodeMode = CanNodeMode::Faulted;
		// 	break;
		// }
		rclcpp::spin_some(n);
		rate.sleep();
	}

	std::cout << "Can Bridge shutting down" << std::endl;
}
