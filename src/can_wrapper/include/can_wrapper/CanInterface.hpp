//Last modified by: Marcin Walczyk/12.10.2023 

#ifndef CANINTERFACE_H
#define CANINTERFACE_H

#include <string>
#include <memory>
#include <exception>
#include <linux/can.h>
#include <net/if.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/fcntl.h>
#include <linux/socket.h>
#include <linux/can/raw.h>
#include <stdint-gcc.h>
#include "can_wrapper/AllCanMessages.hpp"
#include "can_wrapper/exceptions/CreateCanInterfaceFailedException.hpp"

class CanInterface
{
public:
    CanInterface(std::string interfaceName = "can0");
    ~CanInterface();

    int sendRawCanMessage(RawCanMessage rawCanMessage);
    int sendCanMessage(CanMessage canMessage);
    std::unique_ptr<CanMessage> reciveCanMessage();
private:
    sockaddr_can mSockAddrCan;
    socklen_t mSockAddrCanLen;
    int mSocketID;
};

#endif