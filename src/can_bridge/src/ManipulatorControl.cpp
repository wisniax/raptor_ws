#include "can_bridge/ManipulatorControl.hpp"

ManipulatorControl::ManipulatorControl(rclcpp::Node::SharedPtr &nh) : mNh(nh)
{
	const rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(256));

	mRawCanPub = mNh->create_publisher<can_msgs::msg::Frame>(RosCanConstants::RosTopics::can_raw_TX, qos);
	mManipulatorCtlSub = mNh->create_subscription<rex_interfaces::msg::ManipulatorControl>(
		RosCanConstants::RosTopics::can_manipulator_ctl, qos, 
		std::bind(&ManipulatorControl::handleManipulatorCtl, this, std::placeholders::_1));
}
void ManipulatorControl::handleManipulatorCtl(const rex_interfaces::msg::ManipulatorControl::ConstSharedPtr& manipulatroCtlMsg)
{
	// 7 since there are 6 axes + 1 gripper
	std::array<can_msgs::msg::Frame, 7> sendQueue;
	auto sendQueueIter = sendQueue.begin();

	*sendQueueIter++ = encodeStepper((*manipulatroCtlMsg).axes[0], RosCanConstants::VescIds::manipulator_axis_1);
	*sendQueueIter++ = encodeStepper((*manipulatroCtlMsg).axes[1], RosCanConstants::VescIds::manipulator_axis_2);
	*sendQueueIter++ = encodeStepper((*manipulatroCtlMsg).axes[2], RosCanConstants::VescIds::manipulator_axis_3);
	*sendQueueIter++ = encodeStepper((*manipulatroCtlMsg).axes[3], RosCanConstants::VescIds::manipulator_axis_4);
	*sendQueueIter++ = encodeStepper((*manipulatroCtlMsg).axes[4], RosCanConstants::VescIds::manipulator_axis_5);
	*sendQueueIter++ = encodeStepper((*manipulatroCtlMsg).axes[5], RosCanConstants::VescIds::manipulator_axis_6);
	*sendQueueIter++ = encodeStepper((*manipulatroCtlMsg).gripper, RosCanConstants::VescIds::manipulator_gripper);

	for (auto iter = sendQueue.begin(); iter < sendQueue.end(); iter++)
		mRawCanPub->publish(*iter);
}

can_msgs::msg::Frame ManipulatorControl::encodeStepper(const rex_interfaces::msg::VescMotorCommand &stepper, const VESC_Id_t vescId)
{
	VESC_CommandFrame vesc_cf;
	VESC_ZeroMemory(&vesc_cf, sizeof(vesc_cf));
	VESC_RawFrame vesc_rf;
	VESC_ZeroMemory(&vesc_rf, sizeof(vesc_rf));

	vesc_cf.vescID = vescId;
	vesc_cf.command = stepper.command_id;
	vesc_cf.commandData = stepper.set_value;

	VESC_convertCmdToRaw(&vesc_rf, &vesc_cf);
	return VescInterop::vescToRos(vesc_rf);
}