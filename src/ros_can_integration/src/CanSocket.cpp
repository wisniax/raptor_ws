#include "ros_can_integration/CanSocket.hpp"

CanSocket::CanSocket(std::string interfaceName)
{
	mInterfaceName = interfaceName;
	createSocket();
	ROS_INFO_STREAM_COND(mInitErrCode == 0, "CAN: " << translateInitError());
	ROS_ERROR_STREAM_COND(mInitErrCode != 0, "CAN: " << translateInitError());
}

CanSocket::~CanSocket()
{
	if (mInitErrCode != 0)
		return;
	close(mSocket);
}

int CanSocket::createSocket()
{
	mSocket = socket(PF_CAN, SOCK_RAW, CAN_RAW);
	if (mSocket < 0)
	{
		mInitErrCode = -1;
		return mInitErrCode;
	}

	ifreq ifr;
	strcpy(ifr.ifr_name, mInterfaceName.c_str());
	if (ioctl(mSocket, SIOCGIFINDEX, &ifr) < 0)
	{
		mInitErrCode = -2;
		return mInitErrCode;
	}

	struct timeval tv;
	tv.tv_sec = 1;
	tv.tv_usec = 0;
	setsockopt(mSocket, SOL_SOCKET, SO_RCVTIMEO, (const char *)&tv, sizeof tv);

	sockaddr_can addr;
	addr.can_family = AF_CAN;
	addr.can_ifindex = ifr.ifr_ifindex;

	can_err_mask_t err_mask = (CAN_ERR_MASK);

	if (setsockopt(mSocket, SOL_CAN_RAW, CAN_RAW_ERR_FILTER, &err_mask, sizeof(err_mask)) < 0)
	{
		mInitErrCode = -4;
		return mInitErrCode;
	}

	if (bind(mSocket, (struct sockaddr *)&addr, sizeof(addr)) < 0)
	{
		mInitErrCode = -5;
		return mInitErrCode;
	}
	mInitErrCode = 0;
	return mInitErrCode;
}

void CanSocket::setFilter(canid_t id, canid_t mask)
{
	if (mInitErrCode != 0)
		return;
	can_filter rfilter;
	rfilter.can_id = id;
	rfilter.can_mask = CAN_EFF_MASK & mask;
	if (setsockopt(mSocket, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter)) < 0)
	{
		mInitErrCode = -3;
		return;
	}
}

int CanSocket::sendMessage(canid_t frame_id, uint8_t frame_len, uint8_t raw[CAN_MAX_DLEN])
{
	if (mInitErrCode != 0)
		return -2;

	can_frame frame;
	frame.can_id = frame_id;
	frame.can_dlc = frame_len;
	memcpy(frame.data, raw, frame_len);

	return sendMessage(frame);
}

int CanSocket::sendMessage(const can_frame &frame)
{
	if (mInitErrCode != 0)
		return -2;

	ssize_t nbytes = write(mSocket, &frame, sizeof(struct can_frame));
	if (nbytes < 0)
	{
		ROS_ERROR("Can raw socket write failed");
		return -1;
	}

	return nbytes;
}

ssize_t CanSocket::awaitMessage(can_frame &frame)
{
	if (mInitErrCode != 0)
		return -2;
	ssize_t nbytes = read(mSocket, &frame, sizeof(struct can_frame));

	if (nbytes < 0)
	{
		ROS_ERROR("Can raw socket read failed");
		return -1;
	}

	/* paranoid check ... */
	if (nbytes < sizeof(struct can_frame))
	{
		ROS_WARN("Incomplete CAN frame");
		return -1;
	}
	return nbytes;
}

void CanSocket::handleRosCallback(const can_msgs::Frame::ConstPtr &msg)
{
	if (mInitErrCode != 0)
		return;

	can_frame cMsg;
	cMsg.can_id = msg->id;
	cMsg.can_dlc = msg->dlc;
	memcpy(&cMsg.data, msg->data.data(), CAN_MAX_DLEN);

	sendMessage(cMsg);
}

int CanSocket::tryHandleError()
{
	if (mInitErrCode == 0)
		return 0;
	createSocket();
	ROS_INFO_STREAM_COND(mInitErrCode == 0, "CAN: " << translateInitError());
	ROS_ERROR_STREAM_COND(mInitErrCode != 0, "CAN: " << translateInitError());
	ros::Duration(5).sleep();
	return mInitErrCode;
}

int CanSocket::getErrorCode()
{
	return mInitErrCode;
}

std::string CanSocket::translateInitError()
{
	switch (mInitErrCode)
	{
	case 0:
		return "Everything's fineee. Can was properly initialized";
	case -1:
		return "Socket creation failed";
	case -2:
		return "Interface index request failed";
	case -3:
		return "Setting socket's mask failed";
	case -4:
		return "Setting socket's error mask failed";
	case -5:
		return "Binding socket failed";
	default:
		return "I dunno smth's wrong";
	}
}