#include "can_bridge/ManipulatorControl.hpp"

ManipulatorControl::ManipulatorControl(const rclcpp::NodeOptions & options) : Node("manipulator_control", options)
{
	const rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(256));

    mLastRoverStatus = std::make_shared<const RoverStatusMsg>();
    mLastBatteryInfo = std::make_shared<const BatteryInfoMsg>();

    mRawCanPub = this->create_publisher<CanFrame>(RosCanConstants::RosTopics::can_raw_TX, qos);

    mManipulatorCtlSub = this->create_subscription<ManipulatorControlMsg>(
            RosCanConstants::RosTopics::can_manipulator_ctl, qos,
		    std::bind(&ManipulatorControl::handleManipulatorCtl, this, std::placeholders::_1));

    mRoverStatusSub = this->create_subscription<RoverStatusMsg>(
            RosCanConstants::RosTopics::mqtt_rover_status, qos,
            std::bind(&ManipulatorControl::handleRoverStatus, this, std::placeholders::_1));

    mBatteryInfoSub = this->create_subscription<BatteryInfoMsg>(
            RosCanConstants::RosTopics::can_battery_info, qos,
            std::bind(&ManipulatorControl::handleBatteryInfo, this, std::placeholders::_1));
}

void ManipulatorControl::handleManipulatorCtl(const ManipulatorControlMsg::ConstSharedPtr& manipulatorCtlMsg)
{
    if (!isManipulatorMode(mLastRoverStatus))
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5 * 60 * 1000, // Throttle duration (5 minutes)
                             "When non-manipulator mode is selected, incoming ManipulatorControl messages are discarded.");
        return;
    }

    // 7 since there are 6 axes + 1 gripper
    std::array<CanFrame, 7> sendQueue;
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

void ManipulatorControl::handleRoverStatus(const RoverStatusMsg::ConstSharedPtr &roverStatusMsg)
{
    mLastRoverStatus = roverStatusMsg;

    if (!isManipulatorMode(roverStatusMsg) && isManipulatorMode(mLastRoverStatus))
        stopManipulator();
}

void ManipulatorControl::handleBatteryInfo(const BatteryInfoMsg::ConstSharedPtr &msg)
{
    mLastBatteryInfo = msg;
}

bool ManipulatorControl::isManipulatorMode(const RoverStatusMsg::ConstSharedPtr &msg)
{
    // Black Mushroom
    if (mLastBatteryInfo->hotswap_status & BatteryInfoMsg::DRIVE_STOP)
    {
        return false;
    }

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

    // ROBOTIC_ARM, ROBOTIC_ARM_AUTONOMY
    return mode & (RoverStatusMsg::CONTROL_MODE_ROBOTIC_ARM |
                   RoverStatusMsg::CONTROL_MODE_ROBOTIC_ARM_AUTONOMY);
}

void ManipulatorControl::stopManipulator()
{
    ManipulatorControlMsg stopCommand{};

    stopCommand.axes[0].command_id = VESC_COMMAND_SET_DUTY;
    stopCommand.axes[1].command_id = VESC_COMMAND_SET_DUTY;
    stopCommand.axes[2].command_id = VESC_COMMAND_SET_DUTY;
    stopCommand.axes[3].command_id = VESC_COMMAND_SET_DUTY;
    stopCommand.axes[4].command_id = VESC_COMMAND_SET_DUTY;
    stopCommand.axes[5].command_id = VESC_COMMAND_SET_DUTY;
    stopCommand.gripper.command_id = VESC_COMMAND_SET_DUTY;

    stopCommand.axes[0].set_value = 0.0f;
    stopCommand.axes[1].set_value = 0.0f;
    stopCommand.axes[2].set_value = 0.0f;
    stopCommand.axes[3].set_value = 0.0f;
    stopCommand.axes[4].set_value = 0.0f;
    stopCommand.axes[5].set_value = 0.0f;
    stopCommand.gripper.set_value = 0.0f;

    // 7 since there are 6 axes + 1 gripper
    std::array<CanFrame, 7> sendQueue;
    auto sendQueueIter = sendQueue.begin();

    *sendQueueIter++ = encodeStepper((*stopCommand).axes[0], RosCanConstants::VescIds::manipulator_axis_1);
    *sendQueueIter++ = encodeStepper((*stopCommand).axes[1], RosCanConstants::VescIds::manipulator_axis_2);
    *sendQueueIter++ = encodeStepper((*stopCommand).axes[2], RosCanConstants::VescIds::manipulator_axis_3);
    *sendQueueIter++ = encodeStepper((*stopCommand).axes[3], RosCanConstants::VescIds::manipulator_axis_4);
    *sendQueueIter++ = encodeStepper((*stopCommand).axes[4], RosCanConstants::VescIds::manipulator_axis_5);
    *sendQueueIter++ = encodeStepper((*stopCommand).axes[5], RosCanConstants::VescIds::manipulator_axis_6);
    *sendQueueIter++ = encodeStepper((*stopCommand).gripper, RosCanConstants::VescIds::manipulator_gripper);

    for (auto iter = sendQueue.begin(); iter < sendQueue.end(); iter++)
        mRawCanPub->publish(*iter);
}

CanFrame ManipulatorControl::encodeStepper(const rex_interfaces::msg::VescMotorCommand &stepper, const VESC_Id_t vescId)
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
