#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <ros/ros.h>
#include <string>
#include <can_msgs/Frame.h>
#include <memory>
#include "can_wrapper/RosCanConstants.hpp"
#include "can_wrapper/Wheels.h"
#include "can_wrapper/VescCan/CanFrame.hpp"
#include "can_wrapper/VescCan/CanFrameFactory.hpp"

/**
 * @brief Class for interfacing ROS with CAN bus.
 */
class MotorControl
{
public:
	MotorControl(const ros::NodeHandle &nh);

	void sendMotorVel(const can_wrapper::Wheels msg);

private:
	can_msgs::Frame encodeMotorVel(const float msg, const VescCan::Consts::Command command, const uint8_t vescId);

	void handleSetMotorVel(const can_wrapper::Wheels &msg);

	ros::NodeHandle mNh;
	uint32_t mSetMotorVelSeq; /**< Sequence number for motor velocity messages. */

	ros::Publisher mRawCanPub;		 /**< ROS publisher for raw CAN messages. */
	ros::Subscriber mSetMotorVelSub; /**< ROS subscriber for motor velocity messages. */
};

#endif // MOTOR_CONTROL_H