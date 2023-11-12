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

#include "can_wrapper/CanMessage.hpp"

class CanSocket
{
public:
	/// @brief Creates a managed can socket
	/// @param interfaceName A CAN (network) interface name.
	/// @param mask A CAN filter mask.
	CanSocket(std::string interfaceName = "can0");
	~CanSocket();
	int createSocket();
	void setFilter(canid_t id, canid_t mask);
	/// @brief Sends message over CAN via socket
	/// @param frame A whole frame that is ready to send.
	/// @param frame_id CAN message ID. Beware, it also represents priority.
	/// @param frame_len Payload lenght in bytes. Up to 8.
	/// @param raw Payload
	/// @return The number written, or -1 on error
	int sendMessage(canid_t frame_id, uint8_t frame_len, uint8_t raw[CAN_MAX_DLEN]);
	int sendMessage(const CanMessage &frame);
	ssize_t awaitMessage(CanMessage &frame);
	void handleRosCallback(const can_msgs::Frame::ConstPtr &msg);
	int tryHandleError();
	int getErrorCode();
	std::string translateInitError();

private:
	std::string mInterfaceName;
	int mSocket;
	int mInitErrCode;
};

#endif