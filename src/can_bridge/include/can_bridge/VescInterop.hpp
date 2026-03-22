#include <linux/can.h>
#include "can_msgs/msg/frame.hpp"
#include <memory>
extern "C"
{
#include <libVescCan/VESC_Structs.h>
}

namespace VescInterop
{
    can_msgs::msg::Frame vescToRos(const VESC_RawFrame& rawFrame);

    VESC_RawFrame rosToVesc(const can_msgs::msg::Frame& canFrame);
}