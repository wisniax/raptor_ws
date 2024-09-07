#ifndef PROBE_CONTROL_HPP
#define PROBE_CONTROL_HPP

#include <ros/ros.h>
#include <string>
#include <can_msgs/Frame.h>
#include <memory>
#include <array>

#include <can_wrapper/VescInterop.hpp>
#include "can_wrapper/RosCanConstants.hpp"
#include "can_wrapper/SamplerMessage.h"
#include "can_wrapper/RoverStatus.h"

extern "C"
{
#include <libVescCan/VESC.h>
}

class ProbeControl
{
public:
	ProbeControl(const ros::NodeHandle &nh, bool sendOnUpdate = true);

	void sendProbeControl(const can_wrapper::SamplerMessage &msg);
	void sendProbeControl();

private:
	can_msgs::Frame encodeProbeControl(const float msg, const VESC_Command command, const VESC_Id_t vescId);
	void handleProbeControl(const can_wrapper::SamplerMessage &msg);
	void handleRoverStatus(const can_wrapper::RoverStatus &msg);

	ros::NodeHandle mNh;
	uint32_t mProbeControlSeq;						 /**< Sequence number for communication status messages. */
	bool mSendOnUpdate;								 /**< Flag to send messages on update. */
	can_wrapper::SamplerMessage mLastSamplerMessage; /**< Last status message received. */
	can_wrapper::RoverStatus mRoverStatus;

	ros::Publisher mRawCanPub;		  /**< ROS publisher for raw CAN messages. */
	ros::Subscriber mProbeControlSub; /**< ROS subscriber for communication status messages. */
	ros::Subscriber mRoverStatusSub;
};

#endif // PROBE_CONTROL_HPP