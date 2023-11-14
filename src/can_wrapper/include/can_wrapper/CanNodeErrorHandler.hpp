#ifndef CAN_NODE_ERROR_HANDLER_HPP
#define CAN_NODE_ERROR_HANDLER_HPP

#include <ros/ros.h>
#include <queue>
#include <can_msgs/Frame.h>
#include "can_wrapper/RosCanConstants.hpp"
#include "can_wrapper/CanMessage.hpp"
#include "can_wrapper/CanNodeSettingsProvider.hpp"

class CanNodeErrorHandler
{
public:
	enum MaxNumberOfParams : uint8_t
	{
		Rpm_Scale_Params = 2,
		Motor_Reg_Params = 0,
	};

	static void init();

private:
	static bool sIsInitialized;
	static ros::NodeHandle sNh;
	static ros::Subscriber sRawCanSub;
	static ros::Publisher sCanRawPub;

	CanNodeErrorHandler() = delete; // Prevent instantiation

	static void handleRosCallback(const can_msgs::Frame::ConstPtr &msg);
	static void handleErrorFrame(CanMessage cmErr);
	static void handleError(uint8_t dev_id, uint8_t err, CanNodeSettingsProvider::TypeGroups err_group, MaxNumberOfParams iter_count);

	static can_msgs::Frame createResponseFrame(uint8_t dev_id, uint8_t rpm_scale_err);
};

#endif // CAN_NODE_ERROR_HANDLER_HPP