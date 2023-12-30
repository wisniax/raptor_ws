#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <ros/ros.h>
#include <string>
#include <can_msgs/Frame.h>
#include <memory>
#include "can_wrapper/RosCanConstants.hpp"
#include "can_wrapper/Wheels.h"
#include "CM/CM.h"

/**
 * @brief Class for interfacing ROS with CAN bus.
 */
class MotorControl
{
public:
	/**
	 * @brief Initializes the MotorControl object.
	 * @param rpmScale The scale factor for motor RPM.
	 * @param mode The control mode for the motors. Defaults to RPM.
	 */
	MotorControl(float rpmScale, CM_SetMotorVel_ContMode mode = CM_SETMOTORVEL_CONTMODE_RPM);

	/**
	 * @brief Sets the scale factor for motor RPM.
	 * @param rpmScale The scale factor for motor RPM.
	 */
	void setRPMscale(float rpmScale);

	/**
	 * @brief Sends a motor velocity message to the CAN bus.
	 * @param msg The motor velocity message.
	 */
	void sendMotorVel(const can_wrapper::Wheels msg);

	/**
	 * @brief Sets the control mode for the motors.
	 * @param mode The control mode.
	 */
	void setControlMode(CM_SetMotorVel_ContMode mode);

private:
	/**
	 * @brief Encodes motor velocity message into a CAN frame.
	 * @param msg The motor velocity message.
	 * @param adr The CAN address of the motor.
	 * @return The encoded CAN frame.
	 */
	can_msgs::Frame encodeMotorVel(const float msg[], const CM_Address_t adr);

	void setMotorVel(const float msg[], const CM_Address_t adr);

	/**
	 * @brief Callback function for handling left driver velocity messages.
	 * @param msg The left driver velocity message.
	 */
	void handleSetMotorVel(const can_wrapper::Wheels &msg);

	ros::NodeHandle mNh;
	float mRPM_scale;		 /**< Scale factor for motor RPM. */
	uint32_t mSetMotorVelSeq; /**< Sequence number for motor velocity messages. */

	CM_SetMotorVel_ContMode mControlMode; /**< The mode to use when setting motor velocity. */

	ros::Publisher mRawCanPub;		/**< ROS publisher for raw CAN messages. */
	ros::Subscriber mSetMotorVelSub; /**< ROS subscriber for motor velocity messages. */
};

#endif // MOTOR_CONTROL_H