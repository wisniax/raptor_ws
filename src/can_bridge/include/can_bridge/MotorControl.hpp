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
#include "rex_interfaces/msg/rover_status.hpp"
#include "rex_interfaces/msg/battery_info.hpp"
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

	rex_interfaces::msg::Wheels::ConstSharedPtr GetLastSentFrame() const;

private:
	enum State
	{
		DriveStop,
		PrepDriving,
		EStop,
		Driving
	};

	can_msgs::msg::Frame encodeMotorVel(const rex_interfaces::msg::VescMotorCommand &vescMotorCommand, const VESC_Id_t vescId);

	void handleSetMotorVel(const rex_interfaces::msg::Wheels::ConstSharedPtr &msg);
	void handleBatteryInfo(const rex_interfaces::msg::BatteryInfo::ConstSharedPtr &msg);
	void handleRoverStatus(const rex_interfaces::msg::RoverStatus::ConstSharedPtr &msg);
	void stopMotors();
	void setWheelsOrigin();
	void setCorrectState();
	void handleTimerClb();

	rclcpp::Node::SharedPtr mNh;

	rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr mRawCanPub;				  /**< ROS2 publisher for raw CAN messages. */
	rclcpp::Subscription<rex_interfaces::msg::Wheels>::SharedPtr mSetMotorVelSub; /**< ROS2 subscriber for motor velocity messages. */
	rclcpp::Subscription<rex_interfaces::msg::BatteryInfo>::SharedPtr mBatteryInfoSub;
	rclcpp::Subscription<rex_interfaces::msg::RoverStatus>::SharedPtr mRoverStatusSub;

	rex_interfaces::msg::Wheels::ConstSharedPtr mLastSentFrame;
	rex_interfaces::msg::BatteryInfo::ConstSharedPtr mLastBatteryInfo;
	rex_interfaces::msg::RoverStatus::ConstSharedPtr mLastRoverStatus;

	State mState;
	rclcpp::TimerBase::SharedPtr mTimer;
	uint8_t mSetWheelsOriginCtd;
};

#endif // MOTOR_CONTROL_H