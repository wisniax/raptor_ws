#include <can_bridge/VescInterop.hpp>

can_msgs::msg::Frame VescInterop::vescToRos(const VESC_RawFrame& vescf)
{
    //trust me i'm an engineer
    //well, i'm not yet
    can_frame canf;
    memcpy(&canf, &vescf, sizeof(can_frame));

    can_msgs::msg::Frame rosf;

    rosf.id = canf.can_id & CAN_EFF_MASK;
    rosf.is_extended = (canf.can_id & CAN_EFF_FLAG) == CAN_EFF_FLAG;
    rosf.is_rtr = (canf.can_id & CAN_RTR_FLAG) == CAN_RTR_FLAG;
    rosf.is_error = (canf.can_id & CAN_ERR_FLAG) == CAN_ERR_FLAG;
    
    rosf.dlc = canf.can_dlc;
    memcpy(rosf.data.begin(), canf.data, canf.can_dlc);

    return rosf;
}

VESC_RawFrame VescInterop::rosToVesc(const can_msgs::msg::Frame& rosf)
{
    VESC_RawFrame vescf;

    can_frame canf;
    memset(&canf, 0, sizeof(can_frame));

    canf.can_id = rosf.id;
    canf.can_id |= rosf.is_extended ? CAN_EFF_FLAG : 0;
    canf.can_id |= rosf.is_rtr ? CAN_RTR_FLAG : 0;
    canf.can_id |= rosf.is_error ? CAN_ERR_FLAG : 0;

    if (!rosf.is_error && !(rosf.dlc > 8))
    {
        canf.can_dlc = rosf.dlc;
        memcpy(canf.data, rosf.data.begin(), rosf.dlc);
    }

    memcpy(&vescf, &canf, sizeof(can_frame));

    return vescf;
}
