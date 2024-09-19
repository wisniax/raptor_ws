#include "can_wrapper/ManipulatorControl.hpp"

ManipulatorControl::ManipulatorControl(const ros::NodeHandle &nh): 
	mNh(nh)
{
	mRawCanPub = mNh.advertise<can_msgs::Frame>(RosCanConstants::RosTopics::can_raw_TX, 256);
	mManipulatorCtlSub = mNh.subscribe(RosCanConstants::RosTopics::can_manipulator_ctl, 256, &ManipulatorControl::handleManipulatorCtl, this);
}
void ManipulatorControl::handleManipulatorCtl(const can_wrapper::ManipulatorControl::ConstPtr &manipulatroCtlMsg)
{
	//7 since there are 6 axes + 1 gripper
	std::array<can_msgs::Frame, 7> sendQueue;
	auto sendQueueIter = sendQueue.begin();

	*sendQueueIter++ = encodeStepper((*manipulatroCtlMsg).axes[0], RosCanConstants::VescIds::manipulator_axis_1);
	*sendQueueIter++ = encodeStepper((*manipulatroCtlMsg).axes[1], RosCanConstants::VescIds::manipulator_axis_2);
	*sendQueueIter++ = encodeStepper((*manipulatroCtlMsg).axes[2], RosCanConstants::VescIds::manipulator_axis_3);
	*sendQueueIter++ = encodeStepper((*manipulatroCtlMsg).axes[3], RosCanConstants::VescIds::manipulator_axis_4);
	*sendQueueIter++ = encodeStepper((*manipulatroCtlMsg).axes[4], RosCanConstants::VescIds::manipulator_axis_5);
	*sendQueueIter++ = encodeStepper((*manipulatroCtlMsg).axes[5], RosCanConstants::VescIds::manipulator_axis_6);
	*sendQueueIter++ = encodeStepper((*manipulatroCtlMsg).gripper, RosCanConstants::VescIds::manipulator_gripper);

	for(auto iter = sendQueue.begin(); iter < sendQueue.end(); iter++)
		mRawCanPub.publish(*iter);
}

can_msgs::Frame ManipulatorControl::encodeStepper(const can_wrapper::Stepper &stepper, const VESC_Id_t vescId)
{
	VESC_CommandFrame vesc_cf;
	VESC_ZeroMemory(&vesc_cf, sizeof(vesc_cf));
	VESC_RawFrame vesc_rf;
	VESC_ZeroMemory(&vesc_rf, sizeof(vesc_rf));

	vesc_cf.vescID = vescId;
	vesc_cf.command = stepper.commandId;
	vesc_cf.commandData = stepper.setValue;

	VESC_convertCmdToRaw(&vesc_rf, &vesc_cf);
	return VescInterop::vescToRos(vesc_rf);
}