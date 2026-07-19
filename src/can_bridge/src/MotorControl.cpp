#include "can_bridge/MotorControl.hpp"

MotorControl::MotorControl(const rclcpp::NodeOptions & options) : Node("motor_control", options)
{
	const rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(256));

	mState = EStop;
	mSetWheelsOriginCtd = 0;
	mLastBatteryInfo = std::make_shared<const rex_interfaces::msg::BatteryInfo>();
	mLastRoverStatus = std::make_shared<const rex_interfaces::msg::RoverStatus>();

	mRawCanPub = this->create_publisher<can_msgs::msg::Frame>(RosCanConstants::RosTopics::can_raw_TX, qos);

    mSetMotorVelSub = this->create_subscription<rex_interfaces::msg::Wheels>(
		RosCanConstants::RosTopics::can_set_motor_vel,
		qos, std::bind(&MotorControl::handleSetMotorVel, this, std::placeholders::_1));

    mBatteryInfoSub = this->create_subscription<rex_interfaces::msg::BatteryInfo>(
		RosCanConstants::RosTopics::can_battery_info,
		qos, std::bind(&MotorControl::handleBatteryInfo, this, std::placeholders::_1));

    mRoverStatusSub = this->create_subscription<rex_interfaces::msg::RoverStatus>(
		RosCanConstants::RosTopics::mqtt_rover_status,
		qos, std::bind(&MotorControl::handleRoverStatus, this, std::placeholders::_1));

	mTimer = this->create_timer(std::chrono::milliseconds(500), std::bind(&MotorControl::handleTimerClb, this));
}

void MotorControl::handleSetMotorVel(const rex_interfaces::msg::Wheels::ConstSharedPtr &msg)
{
	if (mState != State::Driving)
	{
		RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1 * 60 * 1000, // Throttle duration (1 minute)
							 "Ignoring drive command. Mode is not driving!");
		return;
	}

	mLastSentFrame = msg;
	sendMotorVel(msg);
}

void MotorControl::handleBatteryInfo(const rex_interfaces::msg::BatteryInfo::ConstSharedPtr &msg)
{
	mLastBatteryInfo = msg;
	setCorrectState();
}
void MotorControl::handleRoverStatus(const rex_interfaces::msg::RoverStatus::ConstSharedPtr &msg)
{
	mLastRoverStatus = msg;
	setCorrectState();
}

void MotorControl::stopMotors()
{
	rex_interfaces::msg::Wheels rover_wheels_velocity_temp;

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

	sendMotorVel(std::make_shared<const rex_interfaces::msg::Wheels>(rover_wheels_velocity_temp));
}

void MotorControl::setWheelsOrigin()
{
	rex_interfaces::msg::Wheels rover_wheels_velocity_temp;

	rover_wheels_velocity_temp.header.stamp = this->now();

	rover_wheels_velocity_temp.front_left.drive.command_id = VESC_COMMAND_SET_CURRENT;
	rover_wheels_velocity_temp.front_left.turn.command_id = VESC_COMMAND_SET_ORIGIN;
	rover_wheels_velocity_temp.front_left.turn.set_value = 0.0;
	rover_wheels_velocity_temp.front_left.turn.set_origin_data = 0;

	rover_wheels_velocity_temp.front_right.drive.command_id = VESC_COMMAND_SET_CURRENT;
	rover_wheels_velocity_temp.front_right.turn.command_id = VESC_COMMAND_SET_ORIGIN;
	rover_wheels_velocity_temp.front_right.turn.set_value = 0.0;
	rover_wheels_velocity_temp.front_right.turn.set_origin_data = 0;

	rover_wheels_velocity_temp.rear_right.drive.command_id = VESC_COMMAND_SET_CURRENT;
	rover_wheels_velocity_temp.rear_right.turn.command_id = VESC_COMMAND_SET_ORIGIN;
	rover_wheels_velocity_temp.rear_right.turn.set_value = 0.0;
	rover_wheels_velocity_temp.rear_right.turn.set_origin_data = 0;

	rover_wheels_velocity_temp.rear_left.drive.command_id = VESC_COMMAND_SET_CURRENT;
	rover_wheels_velocity_temp.rear_left.turn.command_id = VESC_COMMAND_SET_ORIGIN;
	rover_wheels_velocity_temp.rear_left.turn.set_value = 0.0;
	rover_wheels_velocity_temp.rear_left.turn.set_origin_data = 0;

	rover_wheels_velocity_temp.front_left.drive.set_value = 0.0;
	rover_wheels_velocity_temp.front_right.drive.set_value = 0.0;
	rover_wheels_velocity_temp.rear_right.drive.set_value = 0.0;
	rover_wheels_velocity_temp.rear_left.drive.set_value = 0.0;

	sendMotorVel(std::make_shared<const rex_interfaces::msg::Wheels>(rover_wheels_velocity_temp));
}

void MotorControl::setCorrectState()
{
    int32_t mode = mLastRoverStatus->control_mode;

    if (mLastRoverStatus->communication_state != mLastRoverStatus->COMMUNICATION_STATE_OPENED)
    {
        stopMotors();
        mState = State::EStop;
        return;
    }

    // NONE, ESTOP
    if (mode == rex_interfaces::msg::RoverStatus::CONTROL_MODE_NONE ||
        (mode & rex_interfaces::msg::RoverStatus::CONTROL_MODE_ESTOP))
    {
        if (mode == 0)
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1 * 60 * 1000, // Throttle duration (1 minute)
                                  "CONTROL_MODE is NONE! Treating as ESTOP.");

        stopMotors();
        mState = State::EStop;
        return;
    }

	if (mLastBatteryInfo->hotswap_status & rex_interfaces::msg::BatteryInfo::DRIVE_STOP)
	{
		mState = State::DriveStop;
		return;
	}

    // STOP
    if (mode & rex_interfaces::msg::RoverStatus::CONTROL_MODE_STOP)
    {
        mState = State::DriveStop;
        return;
    }

    // CONFIG, DRIVE, DRIVE_AUTONOMY
    if (mode & (rex_interfaces::msg::RoverStatus::CONTROL_MODE_DRIVE |
                rex_interfaces::msg::RoverStatus::CONTROL_MODE_DRIVE_AUTONOMY |
                rex_interfaces::msg::RoverStatus::CONTROL_MODE_CONFIG))
    {
        if (mState == State::DriveStop)
        {
            mState = State::Driving;
            return;
        }

        if (mState == State::EStop)
        {
            mState = State::PrepDriving;
            mSetWheelsOriginCtd = 20;
            RCLCPP_INFO(this->get_logger(), "Prepping for driving... Setting cupamars origin.");
        }
        else if (mSetWheelsOriginCtd == 0)
        {
            if (mState == State::PrepDriving) RCLCPP_INFO(this->get_logger(), "Prepping finished.");
            mState = State::Driving;
        }
    }

}

void MotorControl::handleTimerClb()
{
	if (mState != State::PrepDriving)
		return;
	if (mSetWheelsOriginCtd-- != 0)
		setWheelsOrigin();
	setCorrectState();
}

void MotorControl::sendMotorVel(const rex_interfaces::msg::Wheels::ConstSharedPtr &msg)
{
	// 8 since there are 4 wheels, each being vesc + stepper combo
	std::array<can_msgs::msg::Frame, 8> sendQueue;

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

can_msgs::msg::Frame MotorControl::encodeMotorVel(const rex_interfaces::msg::VescMotorCommand &vescMotorCommand, const VESC_Id_t vescId)
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

rex_interfaces::msg::Wheels::ConstSharedPtr MotorControl::GetLastSentFrame() const
{
	return mLastSentFrame;
}

RCLCPP_COMPONENTS_REGISTER_NODE(MotorControl)
