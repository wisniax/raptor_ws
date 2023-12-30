#ifndef CAN_NODE_ERROR_HANDLER_HPP
#define CAN_NODE_ERROR_HANDLER_HPP

#include <ros/ros.h>
#include <queue>
#include <can_msgs/Frame.h>
#include "can_wrapper/RosCanConstants.hpp"
#include "can_wrapper/CanNodeSettingsProvider.hpp"
#include "CM/CM.h"

class CanNodeErrorHandler
{
public:
	enum MaxNumberOfParams : uint8_t
	{
		Rpm_Scale_Params = 2,
		Motor_Reg_Params = 0,
	};

	CanNodeErrorHandler(const std::shared_ptr<const CanNodeSettingsProvider> &canSettingsCPtr);
	void requestDeinitialization();
	void deinitializeDevices();
	void initializeDevices();
	bool GetCanNodesStatus();

private:

	void handleRosCallback(const can_msgs::Frame::ConstPtr &msg);
	void handleErrorFrame(CM_CanMessage cmErr);
	void handleError(uint8_t dev_id, uint8_t err, CM_StmInit_TypeIdFamily err_group, MaxNumberOfParams iter_count);

	can_msgs::Frame createResponseFrame(uint8_t dev_id, uint8_t rpm_scale_err) const;

private:
	ros::NodeHandle mNh;
	ros::Subscriber mRawCanSub;
	ros::Publisher mCanRawPub;
	std::shared_ptr<const CanNodeSettingsProvider> mCanSettings;
	uint8_t mNodeInitCounts = 0;
	bool mNodeOneInitialized = false;
	bool mDeviceInitRequested = false;
	bool mDeinitializationRequested = true;

	// std::map<uint8_t, CanNodeMode> mCanNodes;
};

#endif // CAN_NODE_ERROR_HANDLER_HPP