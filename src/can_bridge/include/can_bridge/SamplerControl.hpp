#ifndef SamplerControl_h_
#define SamplerControl_h_

#include "rclcpp/rclcpp.hpp"
#include <rclcpp_components/register_node_macro.hpp>
#include <string>
#include <array>

#include <can_bridge/VescInterop.hpp>
#include "can_bridge/RosCanConstants.hpp"

#include "can_msgs/msg/frame.hpp"
#include "rex_interfaces/msg/sampler_control.hpp"
#include "rex_interfaces/msg/rover_status.hpp"
#include "rex_interfaces/msg/battery_info.hpp"

extern "C"
{
#include <libVescCan/VESC.h>
}

using SamplerControlMsg = rex_interfaces::msg::SamplerControl;
using RoverStatusMsg = rex_interfaces::msg::RoverStatus;
using BatteryInfoMsg = rex_interfaces::msg::BatteryInfo;
using CanFrame = can_msgs::msg::Frame;

class SamplerControl : public rclcpp::Node
{
public:
	SamplerControl(const rclcpp::NodeOptions & options);

private:
    void handleSamplerCtl(const SamplerControlMsg::ConstSharedPtr &samplerCtlMsg);
    void handleRoverStatus(const RoverStatusMsg::ConstSharedPtr &roverStatusMsg);
    void handleBatteryInfo(const BatteryInfoMsg::ConstSharedPtr &msg);

	bool isSamplerMode(const RoverStatusMsg::ConstSharedPtr &msg);
	void stopSampler();

    void handleTimerClb();
	void publishSamplerData();
	void publish(const VESC_CommandFrame *arr, int arr_size);

	rclcpp::Publisher<CanFrame>::SharedPtr mRawCanPub;	   /**< ROS publisher for raw CAN messages. */
	rclcpp::Subscription<SamplerControlMsg>::SharedPtr mSamplerCtlSub; /**< ROS subscriber for SamplerControl messages. */
	rclcpp::Subscription<RoverStatusMsg>::SharedPtr mRoverStatusSub;   /**< ROS subscriber for SamplerControl messages. */
    rclcpp::Subscription<BatteryInfoMsg>::SharedPtr mBatteryInfoSub;

    SamplerControlMsg::ConstSharedPtr mLastSamplerCtl;
    RoverStatusMsg::ConstSharedPtr mLastRoverStatus;
    BatteryInfoMsg::ConstSharedPtr mLastBatteryInfo;

	rclcpp::TimerBase::SharedPtr mTimer;
	rclcpp::Time mProbeDisableTimestamp;
};

#endif // SamplerControl_h_
