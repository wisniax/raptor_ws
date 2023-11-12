#ifndef ROS2CAN_H
#define ROS2CAN_H

#include <ros/ros.h>
#include <string>
#include <can_msgs/Frame.h>
#include <memory>
#include "can_wrapper/CanMessage.hpp"
#include "can_wrapper/RosCanConstants.hpp"
#include "can_wrapper/Wheels.h"

/**
 * @brief Class for interfacing ROS with CAN bus.
 */
class Ros2Can
{
private:
	ros::NodeHandle nh;						  /**< ROS node handle. */
	float mRPM_scale;						  /**< Scale factor for motor RPM. */
	static std::unique_ptr<Ros2Can> instance; /**< Singleton instance. */
	uint32_t mSetMotorVelSeq;				  /**< Sequence number for motor velocity messages. */

	CanMessage::set_motor_vel_t::mode_cont_mode mControlMode; /**< The mode to use when setting motor velocity. */

	ros::Publisher mRawCanPub;	   /**< ROS publisher for raw CAN messages. */
	ros::Subscriber mSetMotorVelSub;  /**< ROS subscriber for motor velocity messages. */

	/**
	 * @brief Encodes motor velocity message into a CAN frame.
	 * @param msg The motor velocity message.
	 * @param adr The CAN address of the motor.
	 * @return The encoded CAN frame.
	 */
	can_msgs::Frame encodeMotorVel(const float msg[], const CanMessage::Address adr);

	void sendMotorVel(const can_wrapper::Wheels msg);

	/**
	 * @brief Callback function for handling left driver velocity messages.
	 * @param msg The left driver velocity message.
	 */
	static void handleSetMotorVel(const can_wrapper::Wheels &msg);

public:
	/**
	 * @brief Default constructor.
	 */
	Ros2Can() = default;

	/**
	 * @brief Copy constructor (deleted).
	 */
	Ros2Can(Ros2Can &) = delete;

	/**
	 * @brief Assignment operator (deleted).
	 */
	void operator=(const Ros2Can &) = delete;

	/**
	 * @brief Returns the singleton instance of Ros2Can.
	 * @return The singleton instance.
	 */
	static Ros2Can *getSingleton();

	/**
	 * @brief Initializes the Ros2Can object.
	 * @param rpmScale The scale factor for motor RPM.
	 */
	void init(float rpmScale, CanMessage::set_motor_vel_t::mode_cont_mode mode = CanMessage::set_motor_vel_t::mode_cont_mode::TargetModeRpm);

	/**
	 * @brief Sets the scale factor for motor RPM.
	 * @param rpmScale The scale factor for motor RPM.
	 */
	void setRPMscale(float rpmScale);
	
	/**
	 * @brief Sets the control mode for the motors.
	 * @param mode The control mode.
	 */
	void setControlMode(CanMessage::set_motor_vel_t::mode_cont_mode mode);
};

#endif // ROS2CAN_H