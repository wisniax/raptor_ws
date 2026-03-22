#ifndef SamplerControl_h_
#define SamplerControl_h_

#include "rclcpp/rclcpp.hpp"
#include <string>
#include "can_msgs/msg/frame.hpp"
#include <array>
#include <can_bridge/VescInterop.hpp>
#include "can_bridge/RosCanConstants.hpp"
#include "rex_interfaces/msg/sampler_control.hpp"
#include "rex_interfaces/msg/rover_status.hpp"
extern "C"
{
#include <libVescCan/VESC.h>
}

using SamplerControlMsg = rex_interfaces::msg::SamplerControl;
using RoverStatusMsg = rex_interfaces::msg::RoverStatus;

class SamplerControl
{
public:
	SamplerControl(rclcpp::Node::SharedPtr &nh);

private:
	bool isSamplerMode(const RoverStatusMsg::ConstSharedPtr &msg);
	void stopSampler();
	void handleSamplerCtl(const SamplerControlMsg::ConstSharedPtr &samplerCtlMsg);
	void handleRoverStatusClb(const RoverStatusMsg::ConstSharedPtr &roverStatusMsg);
	void handleTimerClb();
	void publishSamplerData();
	void publish(const VESC_CommandFrame *arr, int arr_size);

	rclcpp::Node::SharedPtr mNh;

	rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr mRawCanPub;	   /**< ROS publisher for raw CAN messages. */
	rclcpp::Subscription<SamplerControlMsg>::SharedPtr mSamplerCtlSub; /**< ROS subscriber for SamplerControl messages. */
	rclcpp::Subscription<RoverStatusMsg>::SharedPtr mRoverStatusSub;   /**< ROS subscriber for SamplerControl messages. */

	rclcpp::TimerBase::SharedPtr mTimer;

	rclcpp::Time mProbeDisableTimestamp;

	SamplerControlMsg::ConstSharedPtr mSamplerCtlMsgLast;
	RoverStatusMsg::ConstSharedPtr mRoverStatusMsgLast;
};

#endif // SamplerControl_h_