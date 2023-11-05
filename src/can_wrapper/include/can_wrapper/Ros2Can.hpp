#ifndef ROS2CAN_H
#define ROS2CAN_H

#include <ros/ros.h>
#include <string>
#include <can_msgs/Frame.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <memory>
#include "can_wrapper/CanMessage.hpp"

/**
 * @brief Class for interfacing ROS2 with CAN bus.
 */
class Ros2Can {
private:
	ros::NodeHandle nh; /**< ROS2 node handle. */
	float mRPM_scale; /**< Scale factor for motor RPM. */
	static std::unique_ptr<Ros2Can> instance; /**< Singleton instance. */
	
	/**
	 * @brief Callback function for handling incoming CAN messages.
	 * @param cm The received CAN message.
	 */
	void handleFrame(CanMessage cm);
	
	/**
	 * @brief Encodes motor velocity message into a CAN frame.
	 * @param msg The motor velocity message.
	 * @param adr The CAN address of the motor.
	 * @return The encoded CAN frame.
	 */
	can_msgs::Frame encodeMotorVel(const geometry_msgs::Vector3Stamped msg, const CanMessage::Address adr);
	void sendMotorVel(const geometry_msgs::Vector3Stamped msg, const CanMessage::Address adr);

	/**
	 * @brief Callback function for handling left driver velocity messages.
	 * @param msg The left driver velocity message.
	 */
	static void handleSetLeftDriverVel(const geometry_msgs::Vector3Stamped &msg);
	
	/**
	 * @brief Callback function for handling right driver velocity messages.
	 * @param msg The right driver velocity message.
	 */
	static void handleSetRightDriverVel(const geometry_msgs::Vector3Stamped &msg);
	
	/**
	 * @brief Callback function for handling arm 1-2-3 velocity messages.
	 * @param msg The arm 1-2-3 velocity message.
	 */
	static void handleSetArm123Vel(const geometry_msgs::Vector3Stamped &msg);
	
	/**
	 * @brief Callback function for handling arm 4-5-6 velocity messages.
	 * @param msg The arm 4-5-6 velocity message.
	 */
	static void handleSetArm456Vel(const geometry_msgs::Vector3Stamped &msg);

	can_msgs::Frame canMessage2Frame(CanMessage cm);


	ros::Publisher mRawCanPub; /**< ROS2 publisher for raw CAN messages. */
	ros::Subscriber mDriversLeft; /**< ROS2 subscriber for left driver velocity messages. */
	ros::Subscriber mDriversRight; /**< ROS2 subscriber for right driver velocity messages. */
	ros::Subscriber mArm123; /**< ROS2 subscriber for arm 1-2-3 velocity messages. */
	ros::Subscriber mArm456; /**< ROS2 subscriber for arm 4-5-6 velocity messages. */

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
	 * @param can_RX_topic The ROS2 topic for receiving CAN messages.
	 * @param rpm_scale The scale factor for motor RPM.
	 */
	void init(std::string can_RX_topic = "/CAN/TX/", float rpm_scale = 1);


	/**
	 * @brief Sets the PWM value on a motor.
	 * 
	 * @param msg The PWM value in range -1:1 to set, as a Vector3Stamped message.
	 * @param adr The address of the side to set the PWM value on.
	 * @return uint16_t The status code of the operation.
	 */
	uint16_t setPwmOnMotor(const geometry_msgs::Vector3Stamped msg, const CanMessage::Address adr);

};

#endif // ROS2CAN_H