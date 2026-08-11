#include "can_bridge/ConfigControl.hpp"

ConfigControl::ConfigControl(const rclcpp::NodeOptions & options) : Node("config_control", options)
{
	const rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(256));

	mLastRoverStatus = std::make_shared<const RoverStatusMsg>();
    mLastBatteryInfo = std::make_shared<const BatteryInfoMsg>();

	mRawCanPub = this->create_publisher<CanFrame>(RosCanConstants::RosTopics::can_raw_TX, qos);

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
    if (!isConfigMode(mLastRoverStatus))
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5 * 60 * 1000, // Throttle duration (5 minutes)
                             "Calibration command sent not during CONFIG!");
        return;
    }

    CanFrame fr = encodeMotorVel(*msg, msg->vesc_id);
    mRawCanPub->publish(fr);
}

void ConfigControl::handleRoverStatus(const RoverStatusMsg::ConstSharedPtr &msg)
{
    const bool wasConfigMode = isConfigMode(mLastRoverStatus);
    mLastRoverStatus = msg;

    if (!isConfigMode(msg) && wasConfigMode)
        stopMotors();
}

void ConfigControl::handleBatteryInfo(const BatteryInfoMsg::ConstSharedPtr &msg)
{
    mLastBatteryInfo = msg;
}

bool ConfigControl::isConfigMode(const RoverStatusMsg::ConstSharedPtr &msg)
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

    // CONFIG
    return mode & (RoverStatusMsg::CONTROL_MODE_CONFIG);
}

void ConfigControl::stopMotors()
{
    WheelsMsg rover_wheels_velocity_temp;

    rover_wheels_velocity_temp.header.stamp = this->now();

    rover_wheels_velocity_temp.front_left.turn.command_id = VESC_COMMAND_SET_POS;
    rover_wheels_velocity_temp.front_left.turn.set_value = 0.0;
    rover_wheels_velocity_temp.front_left.turn.set_origin_data = 0;

    rover_wheels_velocity_temp.front_right.turn.command_id = VESC_COMMAND_SET_POS;
    rover_wheels_velocity_temp.front_right.turn.set_value = 0.0;
    rover_wheels_velocity_temp.front_right.turn.set_origin_data = 0;

    rover_wheels_velocity_temp.rear_right.turn.command_id = VESC_COMMAND_SET_POS;
    rover_wheels_velocity_temp.rear_right.turn.set_value = 0.0;
    rover_wheels_velocity_temp.rear_right.turn.set_origin_data = 0;

    rover_wheels_velocity_temp.rear_left.turn.command_id = VESC_COMMAND_SET_POS;
    rover_wheels_velocity_temp.rear_left.turn.set_value = 0.0;
    rover_wheels_velocity_temp.rear_left.turn.set_origin_data = 0;

    rover_wheels_velocity_temp.front_left.drive.command_id = VESC_COMMAND_SET_CURRENT;
    rover_wheels_velocity_temp.front_right.drive.command_id = VESC_COMMAND_SET_CURRENT;
    rover_wheels_velocity_temp.rear_right.drive.command_id = VESC_COMMAND_SET_CURRENT;
    rover_wheels_velocity_temp.rear_left.drive.command_id = VESC_COMMAND_SET_CURRENT;
    rover_wheels_velocity_temp.front_left.drive.set_value = 0.0;
    rover_wheels_velocity_temp.front_right.drive.set_value = 0.0;
    rover_wheels_velocity_temp.rear_right.drive.set_value = 0.0;
    rover_wheels_velocity_temp.rear_left.drive.set_value = 0.0;

    sendMotorVel(std::make_shared<const WheelsMsg>(rover_wheels_velocity_temp));
}

void ConfigControl::sendMotorVel(const WheelsMsg::ConstSharedPtr &msg)
{
    // 8 since there are 4 wheels, each being vesc + stepper combo
    std::array<CanFrame, 8> sendQueue;

    auto sendQueueIter = sendQueue.begin();

    // stepper

    *sendQueueIter++ = encodeMotorVel(
            msg->front_left.turn,
            RosCanConstants::VescIds::front_left_stepper);

    *sendQueueIter++ = encodeMotorVel(
            msg->front_right.turn,
            RosCanConstants::VescIds::front_right_stepper);

    *sendQueueIter++ = encodeMotorVel(
            msg->rear_left.turn,
            RosCanConstants::VescIds::rear_left_stepper);

    *sendQueueIter++ = encodeMotorVel(
            msg->rear_right.turn,
            RosCanConstants::VescIds::rear_right_stepper);

    // vesc

    *sendQueueIter++ = encodeMotorVel(
            msg->front_left.drive,
            RosCanConstants::VescIds::front_left_vesc);

    *sendQueueIter++ = encodeMotorVel(
            msg->front_right.drive,
            RosCanConstants::VescIds::front_right_vesc);

    *sendQueueIter++ = encodeMotorVel(
            msg->rear_left.drive,
            RosCanConstants::VescIds::rear_left_vesc);

    *sendQueueIter++ = encodeMotorVel(
            msg->rear_right.drive,
            RosCanConstants::VescIds::rear_right_vesc);

    // send it ALL

    for (auto iter = sendQueue.begin(); iter < sendQueue.end(); iter++)
        mRawCanPub->publish(*iter);
}

CanFrame ConfigControl::encodeMotorVel(const VescMotorMsg &vescMotorCommand, const VESC_Id_t vescId)
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

    CanFrame fr = VescInterop::vescToRos(rf);
	fr.header.stamp = this->now();

	return fr;
}

RCLCPP_COMPONENTS_REGISTER_NODE(ConfigControl)
