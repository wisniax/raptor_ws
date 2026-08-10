#ifndef BatteryInfoForwarder_h_
#define BatteryInfoForwarder_h_

#include "rclcpp/rclcpp.hpp"
#include <rclcpp_components/register_node_macro.hpp>
#include <boost/shared_ptr.hpp>

#include <can_bridge/RosCanConstants.hpp>
#include <can_bridge/VescInterop.hpp>

#include <can_msgs/msg/frame.hpp>
#include <rex_interfaces/msg/battery_info.hpp>

extern "C"
{
#include <libVescCan/VESC.h>
}

class BatteryInfoForwarder : public rclcpp::Node
{
public:
    BatteryInfoForwarder(const rclcpp::NodeOptions & options);

private:
    void batteryInfoGrabber(const can_msgs::msg::Frame::ConstSharedPtr &frame);
    void batteryInfoPublisher();

    void newVescBatteryInfo(VESC_Status_9 &status9);

    VESC_Status_9 mBatteryInfo;
    bool mBatteryInfoFresh = false;

    // unused at this moment I don't know why it exists
    bool mBatteryWarningIssued = false;

    rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr mBatteryInfoGrabber;
    rclcpp::Publisher<rex_interfaces::msg::BatteryInfo>::SharedPtr mBatteryInfoPublisher;
};

#endif // BatteryInfoForwarder_h_
