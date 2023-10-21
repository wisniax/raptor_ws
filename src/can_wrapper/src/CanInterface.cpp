#include "can_wrapper/CanInterface.hpp"

CanInterface::CanInterface(std::string interfaceName)
{
    struct ifreq interfaceRequest;
    /* open socket */
    mSocketID = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if(mSocketID < 0)
        throw CreateCanInterfaceFailedException("Socket creation failed");

    mSockAddrCan.can_family = AF_CAN;
    strcpy(interfaceRequest.ifr_name, interfaceName.c_str());

    if (ioctl(mSocketID, SIOCGIFINDEX, &interfaceRequest) < 0)
        throw CreateCanInterfaceFailedException("Interface index request failed");

    mSockAddrCan.can_ifindex = interfaceRequest.ifr_ifindex;
    mSockAddrCanLen = sizeof(mSockAddrCan);

    if(fcntl(mSocketID, F_SETFL, O_NONBLOCK))
        throw CreateCanInterfaceFailedException("Socket set flag O_NONBLOCK failed");

    if (bind(mSocketID, (struct sockaddr*)&mSockAddrCan, mSockAddrCanLen) < 0)
        throw CreateCanInterfaceFailedException("Binding socket failed");
}

CanInterface::~CanInterface()
{
    close(mSocketID);
}

int CanInterface::sendRawCanMessage(RawCanMessage rawCanMessage)
{
    can_frame canFrame = rawCanMessage;
    return write(mSocketID,&canFrame,CAN_MTU);
}

int CanInterface::sendCanMessage(CanMessage canMessage)
{
    can_frame canFrame = (RawCanMessage)canMessage;
    return write(mSocketID,&canFrame,CAN_MTU);
}

