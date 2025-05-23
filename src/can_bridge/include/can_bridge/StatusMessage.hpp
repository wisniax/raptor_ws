#ifndef STATUSMESSAGE_HPP_
#define STATUSMESSAGE_HPP_

#include "rclcpp/rclcpp.hpp"
#include <can_msgs/msg/frame.hpp>
#include <rex_interfaces/msg/rover_status.hpp>
#include <can_bridge/VescInterop.hpp>
#include "can_bridge/RosCanConstants.hpp"
extern "C"
{
#include <libVescCan/VESC.h>
}

class StatusMessage
{
public:
	StatusMessage(rclcpp::Node::SharedPtr &nh, bool sendOnUpdate = true);

	void sendStatusMessage(const rex_interfaces::msg::RoverStatus::ConstSharedPtr &msg);
	void sendStatusMessage();

private:
	void handleStatusMessage(const rex_interfaces::msg::RoverStatus::ConstSharedPtr &msg);

	can_msgs::msg::Frame encodeStatusMessage(const rex_interfaces::msg::RoverStatus &msg);

	rclcpp::Node::SharedPtr mNh;

	bool mSendOnUpdate;						  /**< Flag to send messages on update. */
	rex_interfaces::msg::RoverStatus mLastStatus; /**< Last status message received. */

	rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr mRawCanPub;					 /**< ROS2 publisher for raw CAN messages. */
	rclcpp::Subscription<rex_interfaces::msg::RoverStatus>::SharedPtr mStatusMessageSub; /**< ROS2 subscriber for communication status messages. */
};

#endif // STATUSMESSAGE_HPP_