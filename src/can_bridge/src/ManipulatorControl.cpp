#include "can_bridge/ManipulatorControl.hpp"

ManipulatorControl::ManipulatorControl(const rclcpp::NodeOptions & options) : Node("manipulator_control", options)
{
	const rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(256));

	mRawCanPub = this->create_publisher<can_msgs::msg::Frame>(RosCanConstants::RosTopics::can_raw_TX, qos);

    mManipulatorCtlSub = this->create_subscription<rex_interfaces::msg::ManipulatorControl>(
		RosCanConstants::RosTopics::can_manipulator_ctl, qos, 
		std::bind(&ManipulatorControl::handleManipulatorCtl, this, std::placeholders::_1));

    mRoverStatusSub = this->create_subscription<RoverStatusMsg>(
            RosCanConstants::RosTopics::mqtt_rover_status, qos,
            std::bind(&ManipulatorControl::handleRoverStatusClb, this, std::placeholders::_1));

    mRoverStatusMsgLast = std::make_shared<const RoverStatusMsg>();
}

bool ManipulatorControl::isManipulatorMode(const RoverStatusMsg::ConstSharedPtr &msg)
{
    int32_t mode = msg->control_mode;

    if (msg->communication_state != RoverStatusMsg::COMMUNICATION_STATE_OPENED)
    {
        return false;
    }

    // NONE, ESTOP
    if (mode == RoverStatusMsg::CONTROL_MODE_NONE ||
        (mode & RoverStatusMsg::CONTROL_MODE_ESTOP))
    {
        if (mode == 0)
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1 * 60 * 1000, // Throttle duration (1 minute)
                                  "CONTROL_MODE is NONE! Treating as ESTOP.");

        return false;
    }

    // ROBOTIC_ARM, ROBOTIC_ARM_AUTONOMY, CONFIG
    return mode & (RoverStatusMsg::CONTROL_MODE_ROBOTIC_ARM |
                   RoverStatusMsg::CONTROL_MODE_ROBOTIC_ARM_AUTONOMY  |
                   RoverStatusMsg::CONTROL_MODE_CONFIG);
}

void ManipulatorControl::handleRoverStatusClb(const RoverStatusMsg::ConstSharedPtr &roverStatusMsg)
{
    mRoverStatusMsgLast = roverStatusMsg;
}

void ManipulatorControl::handleManipulatorCtl(const rex_interfaces::msg::ManipulatorControl::ConstSharedPtr& manipulatorCtlMsg)
{
    if (!isManipulatorMode(mRoverStatusMsgLast))
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5 * 60 * 1000, // Throttle duration (5 minutes)
                             "When non-manipulator mode is selected, incoming ManipulatorControl messages are discarded.");
        return;
    }

	// 7 since there are 6 axes + 1 gripper
	std::array<can_msgs::msg::Frame, 7> sendQueue;
	auto sendQueueIter = sendQueue.begin();

	*sendQueueIter++ = encodeStepper((*manipulatorCtlMsg).axes[0], RosCanConstants::VescIds::manipulator_axis_1);
	*sendQueueIter++ = encodeStepper((*manipulatorCtlMsg).axes[1], RosCanConstants::VescIds::manipulator_axis_2);
	*sendQueueIter++ = encodeStepper((*manipulatorCtlMsg).axes[2], RosCanConstants::VescIds::manipulator_axis_3);
	*sendQueueIter++ = encodeStepper((*manipulatorCtlMsg).axes[3], RosCanConstants::VescIds::manipulator_axis_4);
	*sendQueueIter++ = encodeStepper((*manipulatorCtlMsg).axes[4], RosCanConstants::VescIds::manipulator_axis_5);
	*sendQueueIter++ = encodeStepper((*manipulatorCtlMsg).axes[5], RosCanConstants::VescIds::manipulator_axis_6);
	*sendQueueIter++ = encodeStepper((*manipulatorCtlMsg).gripper, RosCanConstants::VescIds::manipulator_gripper);

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

RCLCPP_COMPONENTS_REGISTER_NODE(ManipulatorControl)
