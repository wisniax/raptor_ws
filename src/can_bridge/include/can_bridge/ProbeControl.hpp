#ifndef ProbeControl_h_
#define ProbeControl_h_

#include "rclcpp/rclcpp.hpp"
#include <string>
#include "can_msgs/msg/frame.hpp"
#include <array>
#include <can_bridge/VescInterop.hpp>
#include "can_bridge/RosCanConstants.hpp"
#include "can_bridge/msg/probe_control.hpp"
extern "C"
{
#include <libVescCan/VESC.h>
}

class ProbeControl
{
public:
	ProbeControl(rclcpp::Node::SharedPtr &nh);

private:


	void handleProbeCtl(const can_bridge::msg::ProbeControl::ConstSharedPtr &probeCtlMsg);
	void requestProbeStatus();
	void doStuff();
	void publish(const VESC_CommandFrame* arr, int arr_size);

	const VESC_Id_t ID_C1_C2 = 0x83;
	const VESC_Id_t ID_C0 = 0x84;
	
	rclcpp::Node::SharedPtr mNh;

	rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr mRawCanPub;							 /**< ROS publisher for raw CAN messages. */
	rclcpp::Subscription<can_bridge::msg::ProbeControl>::SharedPtr mProbeCtlSub;		 /**< ROS subscriber for ProbeControl messages. */

	rclcpp::TimerBase::SharedPtr mTimerXd;

	can_bridge::msg::ProbeControl::ConstSharedPtr mProbeCtlMsgLast;
};

#endif // ProbeControl_h_