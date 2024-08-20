#include <linux/can.h>
#include <can_msgs/Frame.h>
#include <memory>
extern "C"
{
#include <libVescCan/VESC_Structs.h>
}

namespace VescInterop
{
    can_msgs::Frame vescToRos(const VESC_RawFrame& rawFrame);

    VESC_RawFrame rosToVesc(const can_msgs::Frame& canFrame);
}