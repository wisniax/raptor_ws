#include "rclcpp/rclcpp.hpp"
#include <memory>
#include <linux/can.h>

#include "can_msgs/msg/frame.hpp"
#include "std_msgs/msg/string.hpp"

#include "std_srvs/srv/set_bool.hpp"

#include "can_bridge/MotorControl.hpp"
#include "can_bridge/RosCanConstants.hpp"
#include "can_bridge/VescStatusHandler.hpp"
#include "rex_interfaces/msg/rover_control.hpp"
#include "can_bridge/StatusMessage.hpp"
#include "can_bridge/ManipulatorControl.hpp"
#include "can_bridge/SamplerStatusForwarder.hpp"
#include "can_bridge/SamplerControl.hpp"
#include "can_bridge/BatteryInfoForwarder.hpp"

#include <std_srvs/srv/set_bool.hpp>

enum class CanNodeMode
{
	Created,
	Opening,
	Opened,
	Closing,
	Closed,
	Faulted
};

void doDrivingStuff(MotorControl &mtrCtl);

static std::chrono::system_clock::time_point lastSendWheels;
static std::chrono::nanoseconds diff;

// TODO:
// - Implement this as lifecycle node
// - Utilize a new QoS profile for the publisher and subscriber
// - Recreate the package to use new multi-node architecture (maybe multi-threaded?)
// - Move messages and RosCanConstants to a new package
int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	auto n = rclcpp::Node::make_shared("can_bridge");

	// if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Warn))
	// 	ros::console::notifyLoggerLevelsChanged();

	MotorControl motorControl(n);
	VescStatusHandler mVescStatusHandler(n, &motorControl);
	StatusMessage mStatusMessage(n, true);
	ManipulatorControl mManipulatorCtl(n);
	SamplerStatusForwarder mSamplerStatusForwarder(n);
	SamplerControl mSamplerCtl(n);
	BatteryInfoForwarder mBatteryInfoForwarder(n);

	CanNodeMode canNodeMode = CanNodeMode::Created;
	rclcpp::Rate rate(1000);

	std_srvs::srv::SetBool::Request req;
	std_srvs::srv::SetBool::Response res;

	int iter = 0;

	while (rclcpp::ok())
	{
		rex_interfaces::msg::Wheels vel;
		switch (canNodeMode)
		{
		case CanNodeMode::Created:

			canNodeMode = CanNodeMode::Opening;
			break;

		case CanNodeMode::Opening:
			rclcpp::sleep_for(std::chrono::milliseconds(100));

			canNodeMode = CanNodeMode::Opened;

			break;

		case CanNodeMode::Opened:
			if ((iter++ % 500) == 9)
			{
				mStatusMessage.sendStatusMessage();
			}

			break;

		case CanNodeMode::Closing:
			RCLCPP_INFO(n->get_logger(), "CanNodeMode::Closing");

			canNodeMode = CanNodeMode::Closed;
			break;
		case CanNodeMode::Closed:

			canNodeMode = CanNodeMode::Opening;
			break;
		case CanNodeMode::Faulted:
			rclcpp::sleep_for(std::chrono::seconds(5));
			canNodeMode = CanNodeMode::Created;
			break;
		default:
			RCLCPP_INFO(n->get_logger(), "CanNodeMode::Unknown");
			canNodeMode = CanNodeMode::Faulted;
			break;
		}
		rclcpp::spin_some(n);
		rate.sleep();
	}

	std::cout << "Can Bridge shutting down" << std::endl;
}
