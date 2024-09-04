#ifndef STATUSMESSAGE_HPP_
#define STATUSMESSAGE_HPP_

#include <ros/ros.h>
#include <can_msgs/Frame.h>
#include <can_wrapper/RoverStatus.h>
#include <can_wrapper/VescInterop.hpp>
#include "can_wrapper/RosCanConstants.hpp"
extern "C"
{
#include <libVescCan/VESC.h>
}

class StatusMessage
{
public:
	StatusMessage(const ros::NodeHandle &nh, bool sendOnUpdate = true);

	void sendStatusMessage(const can_wrapper::RoverStatus &msg);
	void sendStatusMessage();

private:
	can_msgs::Frame encodeStatusMessage(const can_wrapper::RoverStatus &msg);

	void handleStatusMessage(const can_wrapper::RoverStatus &msg);

	ros::NodeHandle mNh;
	uint32_t mStatusMessageSeq; /**< Sequence number for communication status messages. */
	bool mSendOnUpdate;		   /**< Flag to send messages on update. */
	can_wrapper::RoverStatus mLastStatus; /**< Last status message received. */

	ros::Publisher mRawCanPub;		   /**< ROS publisher for raw CAN messages. */
	ros::Subscriber mStatusMessageSub; /**< ROS subscriber for communication status messages. */
};

#endif // STATUSMESSAGE_HPP_