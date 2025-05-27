#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "rclcpp/rclcpp.hpp"
#include <string>
#include <can_msgs/msg/frame.hpp>
#include <memory>
#include <array>
#include "can_bridge/VescInterop.hpp"
#include "can_bridge/RosCanConstants.hpp"
#include "rex_interfaces/msg/wheels.hpp"
extern "C"
{
#include <libVescCan/VESC.h>
}



/**
 * @brief Class for interfacing ROS with CAN bus.
 */
class MotorControl
{
public:
	MotorControl(rclcpp::Node::SharedPtr &nh);

	void sendMotorVel(const rex_interfaces::msg::Wheels::ConstSharedPtr &msg);

private:
	can_msgs::msg::Frame encodeMotorVel(const rex_interfaces::msg::VescMotorCommand &vescMotorCommand, const VESC_Id_t vescId);

	void handleSetMotorVel(const rex_interfaces::msg::Wheels::ConstSharedPtr &msg);

	rclcpp::Node::SharedPtr mNh;

	rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr mRawCanPub; /**< ROS2 publisher for raw CAN messages. */
	rclcpp::Subscription<rex_interfaces::msg::Wheels>::SharedPtr mSetMotorVelSub; /**< ROS2 subscriber for motor velocity messages. */
};

#endif // MOTOR_CONTROL_H