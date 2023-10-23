#include "can_wrapper/CanSocket.hpp"

CanSocket::CanSocket(std::string interfaceName, canid_t mask)
{
	mSocket = socket(PF_CAN, SOCK_RAW, CAN_RAW);
	if (mSocket < 0)
	{
		CanSocket::mInitErrCode = -1;
		return;
	}

	ifreq ifr;
	strcpy(ifr.ifr_name, "can0");
	if (ioctl(mSocket, SIOCGIFINDEX, &ifr) < 0)
	{
		CanSocket::mInitErrCode = -2;
		return;
	}

	sockaddr_can addr;
	addr.can_family = AF_CAN;
	addr.can_ifindex = ifr.ifr_ifindex;

	can_filter rfilter;
	rfilter.can_id = 0x0;
	rfilter.can_mask = CAN_EFF_MASK & mask;
	if (setsockopt(mSocket, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter)) < 0)
	{
		CanSocket::mInitErrCode = -3;
		return;
	}

	can_err_mask_t err_mask = (CAN_ERR_MASK);

	if (setsockopt(mSocket, SOL_CAN_RAW, CAN_RAW_ERR_FILTER, &err_mask, sizeof(err_mask)) < 0)
	{
		CanSocket::mInitErrCode = -4;
		return;
	}

	if (bind(mSocket, (struct sockaddr *)&addr, sizeof(addr)) < 0)
	{
		CanSocket::mInitErrCode = -5;
		return;
	}
}

CanSocket::~CanSocket()
{
	if (CanSocket::mInitErrCode != 0)
		return;
	close(mSocket);
}

int CanSocket::sendMessage(canid_t frame_id, uint8_t frame_len, uint8_t raw[CAN_MAX_DLEN])
{
	if (CanSocket::mInitErrCode != 0)
		return -2;

	cm::CanMessage frame;
	frame.address = frame_id;
	frame.dataLength = frame_len;
	memcpy(frame.data.raw, raw, frame_len);

	return CanSocket::sendMessage(frame);
}

int CanSocket::sendMessage(const cm::CanMessage &frame)
{
	if (CanSocket::mInitErrCode != 0)
		return -2;

	ssize_t nbytes = write(mSocket, &frame, sizeof(struct cm::CanMessage));
	if (nbytes < 0)
	{
		ROS_ERROR("Can raw socket write failed");
		return -1;
	}

	return nbytes;
}

ssize_t CanSocket::awaitMessage(cm::CanMessage &frame)
{
	if (CanSocket::mInitErrCode != 0)
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

void CanSocket::handleRosCallback(const std_msgs::String::ConstPtr &msg)
{
	if (CanSocket::mInitErrCode != 0)
		return;

	uint8_t bytom[2] = {0x5, 0xF};

	CanSocket::sendMessage(0x10, 2, bytom);
}

std::string CanSocket::translateInitError()
{
	switch (CanSocket::mInitErrCode)
	{
	case 0:
		return "Everything's fineee";
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