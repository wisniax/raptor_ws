#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "rclcpp/rclcpp.hpp"
#include <string>
#include <can_msgs/msg/frame.hpp>
#include <memory>
#include <array>
#include "can_bridge/VescInterop.hpp"
#include "can_bridge/RosCanConstants.hpp"
#include "can_bridge/msg/wheels.hpp"
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

	void sendMotorVel(const can_bridge::msg::Wheels::ConstSharedPtr &msg);

private:
	can_msgs::msg::Frame encodeMotorVel(const float msg, const VESC_Command command, const VESC_Id_t vescId);

	void handleSetMotorVel(const can_bridge::msg::Wheels::ConstSharedPtr &msg);

	rclcpp::Node::SharedPtr mNh;

	rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr mRawCanPub; /**< ROS2 publisher for raw CAN messages. */
	rclcpp::Subscription<can_bridge::msg::Wheels>::SharedPtr mSetMotorVelSub; /**< ROS2 subscriber for motor velocity messages. */
};

#endif // MOTOR_CONTROL_H