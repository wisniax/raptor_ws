#include "can_bridge/MotorControl.hpp"

ConfigControl::ConfigControl(const rclcpp::NodeOptions & options) : Node("config_control", options)
{
	const rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(256));

	mLastRoverStatus = std::make_shared<const RoverStatusMsg>();
    mLastBatteryInfo = std::make_shared<const BatteryInfoMsg>();

	mRawCanPub = this->create_publisher<can_msgs::msg::Frame>(RosCanConstants::RosTopics::can_raw_TX, qos);

    mCalibrationMotorCommandSub = this->create_subscription<VescMotorMsg>(
            RosCanConstants::RosTopics::can_calibration_motor_command,
            qos, std::bind(&ConfigControl::handleCalibrationMotorCommand, this, std::placeholders::_1));

    mRoverStatusSub = this->create_subscription<RoverStatusMsg>(
		    RosCanConstants::RosTopics::mqtt_rover_status, qos,
            std::bind(&ConfigControl::handleRoverStatus, this, std::placeholders::_1));

    mBatteryInfoSub = this->create_subscription<BatteryInfoMsg>(
            RosCanConstants::RosTopics::can_battery_info, qos,
            std::bind(&ConfigControl::handleBatteryInfo, this, std::placeholders::_1));
}

void ConfigControl::handleCalibrationMotorCommand(const VescMotorMsg::ConstSharedPtr &msg)
{
    if (!isManipulatorMode(mLastRoverStatus))
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5 * 60 * 1000, // Throttle duration (5 minutes)
                             "Calibration command sent not during CONFIG!");
        return;
    }

    can_msgs::msg::Frame fr = encodeMotorVel(*msg, msg->vesc_id);
    mRawCanPub->publish(fr);
}

void ConfigControl::handleRoverStatus(const RoverStatusMsg::ConstSharedPtr &msg)
{
	mLastRoverStatus = msg;
}

void ConfigControl::handleBatteryInfo(const BatteryInfoMsg::ConstSharedPtr &msg)
{
    mLastBatteryInfo = msg;
}

bool ConfigControl::isConfigMode()
{
    // Black Mushroom
    if (mLastBatteryInfo->hotswap_status & BatteryInfoMsg::DRIVE_STOP)
    {
        return false;
    }

    int32_t mode = mLastRoverStatus->control_mode;

    if (mLastRoverStatus->communication_state != RoverStatusMsg::COMMUNICATION_STATE_OPENED)
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

    // CONFIG
    return mode & (RoverStatusMsg::CONTROL_MODE_CONFIG);
}

can_msgs::msg::Frame ConfigControl::encodeMotorVel(const VescMotorMsg &vescMotorCommand, const VESC_Id_t vescId)
{
	VESC_CommandFrame cmdf;
	VESC_ZeroMemory(&cmdf, sizeof(cmdf));

	switch (vescMotorCommand.command_id)
	{
	case VESC_COMMAND_SET_ORIGIN:
		cmdf.commandDataExB = vescMotorCommand.set_origin_data;
		break;
	case VESC_COMMAND_SET_POS_SPEED_LOOP:
		cmdf.commandDataEx_0 = vescMotorCommand.set_pos_speed_loop_position;
		cmdf.commandDataEx_1 = vescMotorCommand.set_pos_speed_loop_speed;
		cmdf.commandDataEx_2 = vescMotorCommand.set_pos_speed_loop_acceleration;
		break;
	default:
		cmdf.commandData = vescMotorCommand.set_value;
	}

	cmdf.command = vescMotorCommand.command_id;
	cmdf.vescID = vescId;

	VESC_RawFrame rf;
	VESC_ZeroMemory(&rf, sizeof(rf));
	VESC_convertCmdToRaw(&rf, &cmdf);

	can_msgs::msg::Frame fr = VescInterop::vescToRos(rf);
	fr.header.stamp = this->now();

	return fr;
}

RCLCPP_COMPONENTS_REGISTER_NODE(ConfigControl)
