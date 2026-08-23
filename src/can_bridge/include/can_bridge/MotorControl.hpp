#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "rclcpp/rclcpp.hpp"
#include <rclcpp_components/register_node_macro.hpp>
#include <string>
#include <memory>
#include <array>

#include "can_bridge/VescInterop.hpp"
#include "ros_constants/RosCanConstants.hpp"

#include <can_msgs/msg/frame.hpp>
#include "rex_interfaces/msg/wheels.hpp"
#include "rex_interfaces/msg/rover_status.hpp"
#include "rex_interfaces/msg/battery_info.hpp"

extern "C"
{
#include <libVescCan/VESC.h>
}

/**
 * @file MotorControl.hpp
 * @brief Header file for the MotorControl component.
 */

using VescMotorMsg = rex_interfaces::msg::VescMotorCommand;
using WheelsMsg = rex_interfaces::msg::Wheels;
using RoverStatusMsg = rex_interfaces::msg::RoverStatus;
using BatteryInfoMsg = rex_interfaces::msg::BatteryInfo;
using CanFrame = can_msgs::msg::Frame;

/**
 * @brief Component responsible for 4-wheel drive and steering control via CAN bus.
 *
 * This class acts as a safety gate between ROS control commands and physical VESC/Cubemars
 * motor controllers. It implements a Finite State Machine (FSM) to handle:
 * - Emergency Stops (E-STOP)
 * - Battery-level based drive inhibition (black mushroom)
 * - Wheel origin calibration (homing) upon escaping black mushroom state.
 *
 * It manages 8 devices on the CAN bus: 4 VESC drive motors and 4 Cubemars steering motors.
 */
class MotorControl : public rclcpp::Node
{
public:
    /**
     * @brief Constructor for the MotorControl component.
     * @param options Configuration options for the composable node.
     */
	MotorControl(const rclcpp::NodeOptions & options);

    /**
     * @brief Processes and sends velocity/angle commands to all 8 motors.
     *
     * Encodes 4 drive commands (current/RPM) and 4 turn commands (position)
     * and publishes them to the raw CAN TX topic.
     *
     * @param msg Shared pointer to the Wheels message containing target states for all wheels.
     */
	void sendMotorVel(const WheelsMsg::ConstSharedPtr &msg);
    /**
     * @brief Returns the last command successfully sent to the motors.
     * @return Const shared pointer to the last sent Wheels message.
     */
    WheelsMsg::ConstSharedPtr GetLastSentFrame() const;

private:
    /**
     * @brief Internal FSM states for drive logic.
     */
	enum State
	{
		DriveStop,   /**< Drive inhibited due to black mushroom. */
		PrepDriving, /**< 5-second calibration phase (sending origin pulses). */
		EStop,       /**< Critical emergency stop - all commands discarded. */
		Driving      /**< Normal operational state - commands are passed to CAN. */
	};

    /**
     * @brief Callback for incoming wheel control messages.
     * Discards commands if internal state is not 'Driving'.
     * @param msg Target wheel velocities and angles.
     */
    void handleSetMotorVel(const WheelsMsg::ConstSharedPtr &msg);
    /**
     * @brief Callback for global rover status updates.
     * Updates internal state based on control mode flags.
     * @param msg Current rover system status.
     */
    void handleRoverStatus(const RoverStatusMsg::ConstSharedPtr &msg);
    /**
     * @brief Callback for battery and BMS information.
     * Monitors the 'drive_stop' for black mushroom.
     * @param msg Battery health and status metrics.
     */
    void handleBatteryInfo(const BatteryInfoMsg::ConstSharedPtr &msg);

    /**
     * @brief Sends a command to all VESC drive motors to set current to 0.0A.
     * Used for safe stopping without active position holding.
     */
	void stopMotors();
    /**
     * @brief Publishes SET_ORIGIN commands to steering motors.
     * Used during the PrepDriving phase.
     */
	void setWheelsOrigin();
    /**
     * @brief Main logic for state transitions.
     * Evaluates RoverStatus, BatteryInfo, and CommunicationState to update mState.
     */
	void setCorrectState();
    /**
     * @brief Timer callback for the 5-second homing procedure.
     * Manages mSetWheelsOriginCtd and triggers setWheelsOrigin().
     */
	void handleTimerClb();

    /**
     * @brief Encodes a high-level motor command into a VESC-compatible CAN frame.
     * @param vescMotorCommand Logic command.
     * @param vescId Target hardware ID on the CAN bus.
     * @return Formatted CAN frame ready for transmission.
     */
    CanFrame encodeMotorVel(const VescMotorMsg &vescMotorCommand, const VESC_Id_t vescId);

    // --- ROS 2 Interfaces ---
    rclcpp::Publisher<CanFrame>::SharedPtr mRawCanPub;	             /**< Publisher for raw CAN messages. */
	rclcpp::Subscription<WheelsMsg>::SharedPtr mSetMotorVelSub;      /**< Subscriber for motor velocity messages. */
	rclcpp::Subscription<RoverStatusMsg>::SharedPtr mRoverStatusSub; /**< Subscriber for system status. */
    rclcpp::Subscription<BatteryInfoMsg>::SharedPtr mBatteryInfoSub; /**< Subscriber for BMS data. */

    // --- Internal State ---
    WheelsMsg::ConstSharedPtr mLastSentFrame;                        /**< Buffer for the last processed wheel command. */
    RoverStatusMsg::ConstSharedPtr mLastRoverStatus;                 /**< Buffer for the last received system status. */
    BatteryInfoMsg::ConstSharedPtr mLastBatteryInfo;                 /**< Buffer for the last received battery info. */

	State mState;                                                    /**< Current internal FSM state. */
    bool mNeedsCalibration = true;                                   /**< Workaround for set origin after black mushroom */
	rclcpp::TimerBase::SharedPtr mTimer;                             /**< Periodic timer for the homing procedure. */
	uint8_t mSetWheelsOriginCtd;                                     /**< Counter for remaining homing pulses. */
};

#endif // MOTOR_CONTROL_H