#include "can_wrapper/CanNodeErrorHandler.hpp"

CanNodeErrorHandler::CanNodeErrorHandler(const std::shared_ptr<const CanNodeSettingsProvider>& canSettingsCPtr)
{
	mCanSettings = canSettingsCPtr;
	mRawCanSub = mNh.subscribe(RosCanConstants::RosTopics::can_raw_RX, 256, &CanNodeErrorHandler::handleRosCallback, this);
	mCanRawPub = mNh.advertise<can_msgs::Frame>(RosCanConstants::RosTopics::can_raw_TX, 256);
}

void CanNodeErrorHandler::handleRosCallback(const can_msgs::Frame::ConstPtr &msg)
{
	if (msg->id & CanMessage::Masks::Adress_Families != (canid_t)CanMessage::Address::Error)
		return;
	handleErrorFrame(CanMessage(msg.get()));
}

void CanNodeErrorHandler::initializeDevices()
{
	if (mNodeOneInitialized || mDeviceInitRequested)
		return;
	CanMessage cm;
	cm.address = CanMessage::Address::Stm_Right | CanMessage::Address::Error | CAN_RTR_FLAG;
	cm.dataLen = 0;
	mCanRawPub.publish((can_msgs::Frame)cm);
	mDeviceInitRequested = true;
}

bool CanNodeErrorHandler::GetCanNodesStatus()
{
	return mNodeOneInitialized;
}

void CanNodeErrorHandler::handleErrorFrame(CanMessage cmErr)
{
	if (cmErr.data.node_errors.select_err & 0b0001)
		handleError(
			cmErr.address & CanMessage::Masks::All_Nodes,
			cmErr.data.node_errors.rpm_scale_err,
			CanNodeSettingsProvider::TypeGroups::Rpm_Scale_Group,
			MaxNumberOfParams::Rpm_Scale_Params);

	if (cmErr.data.node_errors.select_err & 0b0010)
		handleError(
			cmErr.address & CanMessage::Masks::All_Nodes,
			cmErr.data.node_errors.motor_a_reg_err,
			CanNodeSettingsProvider::TypeGroups::Motor_A_Reg_Group,
			MaxNumberOfParams::Motor_Reg_Params);

	if (cmErr.data.node_errors.select_err & 0b0100)
		handleError(
			cmErr.address & CanMessage::Masks::All_Nodes,
			cmErr.data.node_errors.motor_b_reg_err,
			CanNodeSettingsProvider::TypeGroups::Motor_B_Reg_Group,
			MaxNumberOfParams::Motor_Reg_Params);

	if (cmErr.data.node_errors.select_err & 0b1000)
		handleError(
			cmErr.address & CanMessage::Masks::All_Nodes,
			cmErr.data.node_errors.motor_c_reg_err,
			CanNodeSettingsProvider::TypeGroups::Motor_C_Reg_Group,
			MaxNumberOfParams::Motor_Reg_Params);
	
	mDeviceInitRequested = false;
	if (cmErr.data.node_errors.select_err == 0 & cmErr.data.node_errors.unique_err == 0) {
		mNodeOneInitialized = true;
	}
}

void CanNodeErrorHandler::handleError(uint8_t dev_id, uint8_t err, CanNodeSettingsProvider::TypeGroups err_group, MaxNumberOfParams iter_count)
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
	CanMessage cm;
	cm.address = dev_id | CanMessage::Address::Init;
	cm.dataLen = 5;
	cm.data.stm_init.type_id = type_id;
	cm.data.stm_init.var = mCanSettings->getSetting(dev_id, cm.data.stm_init.type_id);
	return (can_msgs::Frame)cm;
}