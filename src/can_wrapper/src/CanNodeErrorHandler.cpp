#include "can_wrapper/CanNodeErrorHandler.hpp"

CanNodeErrorHandler::CanNodeErrorHandler(const std::shared_ptr<const CanNodeSettingsProvider> &canSettingsCPtr)
{
	mCanSettings = canSettingsCPtr;
	mRawCanSub = mNh.subscribe(RosCanConstants::RosTopics::can_raw_RX, 256, &CanNodeErrorHandler::handleRosCallback, this);
	mCanRawPub = mNh.advertise<can_msgs::Frame>(RosCanConstants::RosTopics::can_raw_TX, 256);
}

void CanNodeErrorHandler::handleRosCallback(const can_msgs::Frame::ConstPtr &msg)
{
	if ((msg->id & CM_ADDRESS_FAMILY_MASK) == (uint32_t)CM_ADDRESS_FAMILY_ERROR)
		handleErrorFrame(CM_convert_CanMessage_ftom(msg.get()));
}

void CanNodeErrorHandler::requestDeinitialization()
{
	mDeinitializationRequested = true;
}

void CanNodeErrorHandler::deinitializeDevices()
{
	mNodeOneInitialized = false;
	mDeviceInitRequested = false;
	CM_CanMessage cm;
	cm.address = CM_ADDRESS_STM_RIGHT | CM_ADDRESS_FAMILY_INIT;
	cm.dataLen = 5;
	cm.data.stm_init.type_id = 0;
	cm.data.stm_init.var = 0.0f;
	mCanRawPub.publish(CM_convert_CanMessage_mtof(cm));
}

void CanNodeErrorHandler::initializeDevices()
{
	if (mDeviceInitRequested)
	{
		mNodeInitCounts++;
		if (mNodeInitCounts > 10)
		{
			mNodeInitCounts = 0;
			mDeviceInitRequested = false;
		}
		return;
	}

	if (mDeinitializationRequested)
	{
		deinitializeDevices();
	}

	if (mNodeOneInitialized || mDeviceInitRequested)
		return;

	CM_CanMessage cm;
	cm.address = CM_ADDRESS_STM_RIGHT | CM_ADDRESS_FAMILY_ERROR | CAN_RTR_FLAG;
	cm.dataLen = 0;
	mCanRawPub.publish(CM_convert_CanMessage_mtof(cm));
	ros::spinOnce();
	mDeviceInitRequested = true;
}

bool CanNodeErrorHandler::GetCanNodesStatus()
{
	return mNodeOneInitialized;
}

void CanNodeErrorHandler::handleErrorFrame(CM_CanMessage cmErr)
{
	if (cmErr.data.node_errors.select_err & CM_NODEERRORS_SELECTERR_FLAG_RPMSCALE)
		handleError(
			cmErr.address & CM_ADDRESS_MASK,
			cmErr.data.node_errors.rpm_scale_err,
			CM_STMINIT_TYPEID_FAMILY_MOTORCONTROL,
			MaxNumberOfParams::Rpm_Scale_Params);

	if (cmErr.data.node_errors.select_err & CM_NODEERRORS_SELECTERR_FLAG_MOTORDRIVER_A)
		handleError(
			cmErr.address & CM_ADDRESS_MASK,
			cmErr.data.node_errors.motor_a_reg_err,
			CM_STMINIT_TYPEID_FAMILY_MOTOR_A,
			MaxNumberOfParams::Motor_Reg_Params);

	if (cmErr.data.node_errors.select_err & CM_NODEERRORS_SELECTERR_FLAG_MOTORDRIVER_B)
		handleError(
			cmErr.address & CM_ADDRESS_MASK,
			cmErr.data.node_errors.motor_b_reg_err,
			CM_STMINIT_TYPEID_FAMILY_MOTOR_B,
			MaxNumberOfParams::Motor_Reg_Params);

	if (cmErr.data.node_errors.select_err & CM_NODEERRORS_SELECTERR_FLAG_MOTORDRIVER_C)
		handleError(
			cmErr.address & CM_ADDRESS_MASK,
			cmErr.data.node_errors.motor_c_reg_err,
			CM_STMINIT_TYPEID_FAMILY_MOTOR_C,
			MaxNumberOfParams::Motor_Reg_Params);

	mDeviceInitRequested = false;
	if (cmErr.data.node_errors.select_err == CM_NODEERRORS_SELECTERR_OK & cmErr.data.node_errors.unique_err == CM_NODEERRORS_UNIQUEERR_OK)
	{
		mNodeOneInitialized = true;
	}
	else
		mDeinitializationRequested = false;
}

void CanNodeErrorHandler::handleError(uint8_t dev_id, uint8_t err, CM_StmInit_TypeIdFamily err_group, MaxNumberOfParams iter_count)
{
	if (err != 0xF)
	{
		mCanRawPub.publish(createResponseFrame(dev_id, err | err_group));
		return;
	}

	for (uint8_t i = 0; i < iter_count; i++)
	{
		mCanRawPub.publish(createResponseFrame(dev_id, i | err_group));
	}
}

can_msgs::Frame CanNodeErrorHandler::createResponseFrame(uint8_t dev_id, uint8_t type_id) const
{
	CM_CanMessage cm;
	cm.address = dev_id | CM_ADDRESS_FAMILY_INIT;
	cm.dataLen = 5;
	cm.data.stm_init.type_id = type_id;
	cm.data.stm_init.var = mCanSettings->getSetting(dev_id, cm.data.stm_init.type_id);
	return CM_convert_CanMessage_mtof(cm);
}