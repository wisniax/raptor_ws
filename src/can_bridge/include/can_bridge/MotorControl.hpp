#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "rclcpp/rclcpp.hpp"
#include <rclcpp_components/register_node_macro.hpp>
#include <string>
#include <memory>
#include <array>

#include "can_bridge/VescInterop.hpp"
#include "can_bridge/RosCanConstants.hpp"

#include <can_msgs/msg/frame.hpp>
#include "rex_interfaces/msg/wheels.hpp"
#include "rex_interfaces/msg/rover_status.hpp"
#include "rex_interfaces/msg/battery_info.hpp"

extern "C"
{
#include <libVescCan/VESC.h>
}

using WheelsMsg = rex_interfaces::msg::Wheels;
using RoverStatusMsg = rex_interfaces::msg::RoverStatus;
using BatteryInfoMsg = rex_interfaces::msg::BatteryInfo;

/**
 * @brief Class for interfacing ROS with CAN bus.
 */
class MotorControl : public rclcpp::Node
{
public:
	MotorControl(const rclcpp::NodeOptions & options);

	void sendMotorVel(const WheelsMsg::ConstSharedPtr &msg);
    WheelsMsg::ConstSharedPtr GetLastSentFrame() const;

private:
	enum State
	{
		DriveStop,
		PrepDriving,
		EStop,
		Driving
	};

    void handleSetMotorVel(const WheelsMsg::ConstSharedPtr &msg);
    void handleRoverStatus(const RoverStatusMsg::ConstSharedPtr &msg);
    void handleBatteryInfo(const BatteryInfoMsg::ConstSharedPtr &msg);

	void stopMotors();
	void setWheelsOrigin();
	void setCorrectState();
	void handleTimerClb();

    can_msgs::msg::Frame encodeMotorVel(const rex_interfaces::msg::VescMotorCommand &vescMotorCommand, const VESC_Id_t vescId);

    rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr mRawCanPub;				  /**< ROS2 publisher for raw CAN messages. */
	rclcpp::Subscription<WheelsMsg>::SharedPtr mSetMotorVelSub; /**< ROS2 subscriber for motor velocity messages. */
	rclcpp::Subscription<RoverStatusMsg>::SharedPtr mRoverStatusSub;
    rclcpp::Subscription<BatteryInfoMsg>::SharedPtr mBatteryInfoSub;

    WheelsMsg::ConstSharedPtr mLastSentFrame;
    RoverStatusMsg::ConstSharedPtr mLastRoverStatus;
    BatteryInfoMsg::ConstSharedPtr mLastBatteryInfo;

	State mState;
	rclcpp::TimerBase::SharedPtr mTimer;
	uint8_t mSetWheelsOriginCtd;
};

#endif // MOTOR_CONTROL_H
