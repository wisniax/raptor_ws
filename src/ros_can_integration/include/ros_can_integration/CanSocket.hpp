#ifndef CANSOCKET_H
#define CANSOCKET_H

#include <linux/can.h>
#include <linux/can/raw.h>
#include <linux/can/error.h>
#include <string>
#include <net/if.h>
#include <unistd.h>
#include <ros/ros.h>
#include <sys/ioctl.h>
#include <sys/fcntl.h>
#include <can_msgs/Frame.h>
#include <std_srvs/SetBool.h>

class CanSocket
{
public:
	/// @brief Creates a managed can socket
	/// @param interfaceName A CAN (network) interface name.
	/// @param awaitMessageTimeout A timeout for awaiting for a message in seconds.
	CanSocket(std::string interfaceName, uint32_t awaitMessageTimeoutSec = 0, uint32_t awaitMessageTimeoutUsec = 1000);

	~CanSocket();

	void setFilter(canid_t id, canid_t mask);

	/// @brief Sends message over CAN via socket
	/// @param frame A whole frame that is ready to send.
	/// @param frame_id CAN message ID. Beware, it also represents priority.
	/// @param frame_len Payload lenght in bytes. Up to 8.
	/// @param raw Payload
	/// @return The number of bytes written, or -1, -2 on error
	int sendMessage(canid_t frame_id, uint8_t frame_len, uint8_t raw[CAN_MAX_DLEN]);

	/// @brief Sends message over CAN via socket
	/// @param frame A whole frame that is ready to send.
	/// @return The number of bytes written, or -1, -2 on error
	int sendMessage(const can_frame &frame);

	/**
	 * @brief Awaits for a CAN message with timeout set in constructor.
	 * @param frame A CAN frame to be filled with received data.
	 * @return The number of bytes read, or -1, -2 on error
	 */
	ssize_t awaitMessage(can_frame &frame);

	ssize_t awaitAndPublishCanMessage(ros::Publisher &canRawPub);

	/**
	 * @brief Handles ROS callback
	 * @param msg A received CAN frame.
	 */
	void handleRosCallback(const can_msgs::Frame::ConstPtr &msg);

	/**
	 * @brief Creates a socket
	 * @return Error code
	 */
	int createSocket();

	/**
	 * @brief Gets socket error code
	 * @return Error code
	 */
	int getErrorCode();

	bool getErrorCodeCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);

	/**
	 * @brief Translates socket error code to human readable string
	 * @return Error string
	 */
	std::string translateInitError();

private:
	int tryCreateSocket();
	int tryHandleError();

private:
	size_t mMaxIterCount;
	std::string mInterfaceName;
	int mSocket = 0;
	int mInitErrCode = -69;
	uint32_t mSeqCnt = 0;
	timeval mAwaitMessageTimeout;
	ros::Time mSocketCreatedTimestamp;
	ros::Duration mSocketMinimumLifetime;
};

#endif