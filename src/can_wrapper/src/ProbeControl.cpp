#include "can_wrapper/ProbeControl.hpp"

ProbeControl::ProbeControl(const ros::NodeHandle &nh, bool sendOnUpdate)
{
    can_wrapper::SamplerMessage zeroMsg;
    zeroMsg.DrillCommand = 0;
    zeroMsg.PlatformCommand = 0;
    zeroMsg.DrillState = 0;
    zeroMsg.isContainerExtended = false;
    mLastSamplerMessage = zeroMsg;

    mSendOnUpdate = sendOnUpdate;
    mProbeControlSeq = 0;

    mRawCanPub = mNh.advertise<can_msgs::Frame>(RosCanConstants::RosTopics::can_raw_TX, 256);
    mProbeControlSub = mNh.subscribe("/MQTT/SamplerControl", 256, &ProbeControl::handleProbeControl, this);
    // mRoverStatusSub = mNh.subscribe("/MQTT/RoverStatus", 256, &ProbeControl::handleProbeControl, this);
}

void ProbeControl::handleProbeControl(const can_wrapper::SamplerMessage &msg)
{
    std::array<can_msgs::Frame, 8> sendQueue;

    mLastSamplerMessage = msg;
    if (mSendOnUpdate)
        sendProbeControl(msg);
}

void ProbeControl::handleRoverStatus(const can_wrapper::RoverStatus &msg)
{
    mRoverStatus = msg;
    sendProbeControl(mLastSamplerMessage);
}

void ProbeControl::sendProbeControl(const can_wrapper::SamplerMessage &msg)
{
    mLastSamplerMessage = msg;
    sendProbeControl();
}

void ProbeControl::sendProbeControl()
{
    // if (mRoverStatus.CommunicationState != 2)
    // {
    //     can_wrapper::SamplerMessage zeroMsg;
    //     zeroMsg.DrillCommand = 0;
    //     zeroMsg.DrillState = 0;
    //     zeroMsg.isContainerExtended = false;
    //     zeroMsg.PlatformCommand = 0;
    //     mLastSamplerMessage = zeroMsg;
    // }
    
    std::array<can_msgs::Frame, 4> sendQueue;

    auto sendQueueIter = sendQueue.begin();

    float value = 0.0;

    switch (mLastSamplerMessage.DrillCommand)
    {
    case 0:
        value = 0.0f;
        break;
    case 1:
        value = -0.7f;
        break;
    case 2:
        value = 0.6f;
        break;
    default:
        value = 0.0f;
        break;
    }

    *sendQueueIter++ = encodeProbeControl(
        value,
        static_cast<VESC_Command>(0),
        0x81);

    switch (mLastSamplerMessage.PlatformCommand)
    {
    case 0:
        value = 0.0f;
        break;
    case 1:
        value = -0.05f;
        break;
    case 2:
        value = 0.20f;
        break;
    default:
        value = 0.0f;
        break;
    }

    *sendQueueIter++ = encodeProbeControl(
        value,
        static_cast<VESC_Command>(0),
        0x80);

    switch (mLastSamplerMessage.DrillState)
    {
    case 0:
        value = 0.0f;
        break;
    case 1:
        value = -0.25f;
        break;
    case 2:
        value = 0.15f;
        break;
    case 3:
        value = -0.80f;
        break;
    case 4:
        value = 0.80f;
        break;
    default:
        value = 0.0f;
        break;
    }

    *sendQueueIter++ = encodeProbeControl(
        value,
        static_cast<VESC_Command>(0),
        0x82);

    value = mLastSamplerMessage.isContainerExtended ? 1.0f : -1.0f;

    *sendQueueIter++ = encodeProbeControl(
        mLastSamplerMessage.isContainerExtended,
        static_cast<VESC_Command>(0),
        0x83);

    for (auto iter = sendQueue.begin(); iter < sendQueue.end(); iter++)
        mRawCanPub.publish(*iter);
}

can_msgs::Frame ProbeControl::encodeProbeControl(const float msg, const VESC_Command command, const VESC_Id_t vescId)
{
    VESC_CommandFrame cmdf;
    VESC_ZeroMemory(&cmdf, sizeof(cmdf));

    cmdf.commandData = msg;
    cmdf.command = command;
    cmdf.vescID = vescId;

    VESC_RawFrame rf;
    VESC_ZeroMemory(&rf, sizeof(rf));
    VESC_convertCmdToRaw(&rf, &cmdf);

    can_msgs::Frame fr = VescInterop::vescToRos(rf);
    fr.header.seq = mProbeControlSeq++;
    return fr;
}
