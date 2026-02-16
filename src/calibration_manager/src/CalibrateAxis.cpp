#include "calibration_manager/CalibrateAxis.hpp"

const std::string CALIBRATION_MAX_SPEED = "max_speed";
const std::string CALIBRATION_MAX_OFFSET_SHIFT = "max_offset_shift";
const std::string CALIBRATION_MAX_VELOCITY_SHIFT = "max_velocity_shift";

const std::string CALIBRATION_OUTDATED_DURATION_S = "outdated_duration_s";
const std::string CALIBRATION_SPEED_TIMEOUT_MS = "speed_timeout_ms";
const std::string CALIBRATION_MESSAGE_SEND_PERIOD_MS = "message_send_period_ms";

const std::string CALIBRATION_STOP_TOLERANCE = "stop_tolerance";

const std::string CALIBRATION_LOG_SETPOS_DIFF = "log_setpos_diff";
const std::string CALIBRATION_USE_SCHEDULE_HOLD = "use_schedule_hold";

CalibrateAxis::CalibrateAxis(const rclcpp::NodeOptions &options) : Node("calibrate_axis", options)
{
	const rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(256));

	mCalibrationMotors = {
		RosCanConstants::VescIds::front_left_stepper,
		RosCanConstants::VescIds::front_right_stepper,
		RosCanConstants::VescIds::rear_left_stepper,
		RosCanConstants::VescIds::rear_right_stepper,
	};

	initParams();

	mCalibrationMotorCommandPub = this->create_publisher<rex_interfaces::msg::VescMotorCommand>(
		RosCanConstants::RosTopics::can_calibration_motor_command, qos);
	mVescStatusSub = this->create_subscription<rex_interfaces::msg::VescStatus>(
		RosCanConstants::RosTopics::can_vesc_status,
		qos, std::bind(&CalibrateAxis::handleVescStatus, this, std::placeholders::_1));
	mCalibrateAxisSub = this->create_subscription<rex_interfaces::msg::CalibrateAxis>(
		RosCanConstants::RosTopics::mqtt_calibrate_axis,
		qos, std::bind(&CalibrateAxis::handleCalibrateAxis, this, std::placeholders::_1));
	mRoverStatusSub = this->create_subscription<rex_interfaces::msg::RoverStatus>(
		RosCanConstants::RosTopics::mqtt_rover_status,
		qos, std::bind(&CalibrateAxis::handleRoverStatus, this, std::placeholders::_1));
	mBatteryInfoSub = this->create_subscription<rex_interfaces::msg::BatteryInfo>(
		RosCanConstants::RosTopics::can_battery_info,
		qos, std::bind(&CalibrateAxis::handleBatteryInfo, this, std::placeholders::_1));

	mFrameSender = this->create_timer(
		std::chrono::milliseconds(mIntParams[CALIBRATION_MESSAGE_SEND_PERIOD_MS]),
		std::bind(&CalibrateAxis::sendFrame, this));

	mVelocityTimeoutTimer = this->create_timer(
		std::chrono::milliseconds(mIntParams[CALIBRATION_SPEED_TIMEOUT_MS]),
		std::bind(&CalibrateAxis::timeoutTimerTick, this));
	mVelocityTimeoutTimer->cancel();

	modeNothing();
	mHoldScheduled = false;

	RCLCPP_INFO(this->get_logger(), "Calibration module started.");
};

void CalibrateAxis::initParams()
{
	// Loads node parameters from args, launch files etc.
	// Also registers a callback for dynamic param changing

	mFloatParams = {
		{CALIBRATION_MAX_SPEED, 3.0f},
		{CALIBRATION_MAX_OFFSET_SHIFT, 30.0f},
		{CALIBRATION_MAX_VELOCITY_SHIFT, 30.0f},
		{CALIBRATION_OUTDATED_DURATION_S, 0.3f},
		{CALIBRATION_STOP_TOLERANCE, 0.5f}};
	for (auto &[name, value] : mFloatParams)
	{
		this->declare_parameter(name, value);
		mFloatParams[name] = this->get_parameter(name).as_double();
	}

	mIntParams = {
		{CALIBRATION_SPEED_TIMEOUT_MS, 500},
		{CALIBRATION_MESSAGE_SEND_PERIOD_MS, 1000 / 50},
		{CALIBRATION_LOG_SETPOS_DIFF, 0},
		{CALIBRATION_USE_SCHEDULE_HOLD, 1}};
	for (auto &[name, value] : mIntParams)
	{
		this->declare_parameter(name, value);
		mIntParams[name] = this->get_parameter(name).as_int();
	}

	// mParamCallbackHandle = this->add_post_set_parameters_callback(
	// 	[this](const std::vector<rclcpp::Parameter> &params)
	// 	{
	// 		for (const auto &param : params)
	// 		{
	// 			const std::string name = param.get_name();
	// 			if (mFloatParams.count(name))
	// 			{
	// 				mFloatParams[name] = param.as_double();
	// 			}
	// 			else if (mIntParams.count(name))
	// 			{
	// 				mIntParams[name] = param.as_int();
	// 			}
	// 		}
	// 	});
}

// ######################### MSG HANDLERS #########################

void CalibrateAxis::handleVescStatus(const rex_interfaces::msg::VescStatus::ConstSharedPtr &msg)
{
	// RCLCPP_INFO(this->get_logger(), "%d precise %lf pid %f", msg->vesc_id, msg->precise_pos, msg->pid_pos);
	if (!calibrationMotorsContains(msg->vesc_id))
	{
		// Only interested in calibratable motors.
		return;
	}
	rclcpp::Time now = this->get_clock()->now();
	mMotorStatuses[msg->vesc_id] = {static_cast<float>(msg->precise_pos), msg->erpm, now};

	// Below only applies to the currently-calibrated motor.

	if (msg->vesc_id != mCurrentMotorID)
		return;

	if (mHoldScheduled)
	{
		if (mMode != Mode::Nothing)
		{
			RCLCPP_ERROR(this->get_logger(), "Hold schedule wasn't cancelled correctly! Any mode changes should cancel it.");
		}
		else
		{
			// Now that new status has been received, motor can Hold (prevents jerk when stopping during SetVelocity)
			modeHold(mCurrentMotorID);
			return;
		}
	}

	// --- Mode end-conditions below

	if (mMode == Mode::SetPos && checkSetPosEndCondition(msg))
	{
		RCLCPP_INFO(this->get_logger(), "End of SetPos reached, holding...");
		modeHold(mCurrentMotorID);
		return;
	}

	// If motor is rotated outside the allowed calibration range
	if (
		mMode == Mode::SetVelocity &&
		std::abs(msg->precise_pos) > mFloatParams[CALIBRATION_MAX_VELOCITY_SHIFT] &&
		signum(mFrameToSend.set_value) == signum(msg->precise_pos) // Moving away from origin
	)
	{
		RCLCPP_INFO(this->get_logger(), "Rotated more than is allowed in velocity mode, snapping to %f", signum(msg->precise_pos) * (mFloatParams[CALIBRATION_MAX_VELOCITY_SHIFT] + 1));
		cancelTimeout();
		// Snap to max shift
		// +1 so that SetVelocity frames in the outer direction will still be rejected
		modeSetPos(msg->vesc_id, signum(msg->precise_pos) * (mFloatParams[CALIBRATION_MAX_VELOCITY_SHIFT] + 1));
	}
}

void CalibrateAxis::handleCalibrateAxis(const rex_interfaces::msg::CalibrateAxis::ConstSharedPtr &msg)
{
	using CalibrateMsg = rex_interfaces::msg::CalibrateAxis;

	if (!mLastRoverStatus || mLastRoverStatus->control_mode != rex_interfaces::msg::RoverStatus::CONTROL_MODE_ESTOP)
	{
		RCLCPP_ERROR(this->get_logger(), "Rover not in ESTOP, calibration not permitted.");
		return;
	}
	if (!mLastBatteryInfo || mLastBatteryInfo->hotswap_status & rex_interfaces::msg::BatteryInfo::DRIVE_STOP)
	{
		RCLCPP_ERROR(this->get_logger(), "Black mushroom enganed (or status unknown), calibration not permitted.");
		return;
	}

	if (!calibrationMotorsContains(msg->vesc_id))
	{
		RCLCPP_ERROR(this->get_logger(), "Attempted to calibrate motor with invalid VESC ID: %#x", msg->vesc_id);
		return;
	}

	// If motor is moving by SetVelocity, only STOP, CANCEL and SET_VELOCITY are fine
	// If motor is moving by Offset, only STOP and CANCEL are fine
	// If mMode is Hold or Nothing, all action types are allowed.
	if (mMode == Mode::SetVelocity)
	{
		if (msg->action_type != CalibrateMsg::ACTION_TYPE_SET_VELOCITY &&
			msg->action_type != CalibrateMsg::ACTION_TYPE_STOP &&
			msg->action_type != CalibrateMsg::ACTION_TYPE_CANCEL)
		{
			RCLCPP_INFO(this->get_logger(), "Calibration request rejected, the motor is still moving");
			return;
		}
	}
	else if (mMode == Mode::SetPos)
	{
		if (!(msg->action_type == CalibrateMsg::ACTION_TYPE_STOP || msg->action_type == CalibrateMsg::ACTION_TYPE_CANCEL))
		{
			RCLCPP_INFO(this->get_logger(), "Calibration request rejected, the motor is still moving");
			return;
		}
	}

	// --------------------

	// If a different motor is currently being calibrated,
	// stop it and forget it.
	if (mCurrentMotorID && mCurrentMotorID != msg->vesc_id)
	{
		stopMotor(mCurrentMotorID);
		modeNothing();
	}

	// --------------------

	rex_interfaces::msg::VescMotorCommand fr;
	float offsetShift;
	switch (msg->action_type)
	{
	case CalibrateMsg::ACTION_TYPE_STOP:
		stopMotor(msg->vesc_id);
		modeNothing(); // Clear the SetPos frame so that Hold doesn't use it (for safety reasons)
		if (mIntParams[CALIBRATION_USE_SCHEDULE_HOLD])
			scheduleHold(msg->vesc_id);
		else
			modeHold(msg->vesc_id);
		break;

	case CalibrateMsg::ACTION_TYPE_RETURN_TO_ORIGIN:
		modeSetPos(msg->vesc_id, 0.0f);
		break;

	case CalibrateMsg::ACTION_TYPE_CONFIRM:
		stopMotor(msg->vesc_id);
		fr = frameSetOrigin(msg->vesc_id);
		mCalibrationMotorCommandPub->publish(fr);
		modeNothing();
		break;

	case CalibrateMsg::ACTION_TYPE_CANCEL:
		stopMotor(msg->vesc_id);
		modeNothing();
		break;

	case CalibrateMsg::ACTION_TYPE_OFFSET:
		// If mode was SetPos or Hold, take the starting position from the frame
		// Otherwise, capture position from feedback
		float startingPosition;
		switch (mMode)
		{
		case Mode::SetPos:
		case Mode::Hold:
			// Continue to offset based on previous position.
			// Scale related to https://github.com/AlvaroBajceps/libVescCan/issues/10
			startingPosition = mFrameToSend.set_value * 100.0;
			break;
		default:
			if (!isRecordedStatusValid(msg->vesc_id))
			{
				RCLCPP_ERROR(
					this->get_logger(),
					"No recent motor status recorded - no reference start position, cannot rotate.");
				return;
			}
			startingPosition = mMotorStatuses[msg->vesc_id].position;
		}

		// Limit value
		offsetShift = msg->value;
		if (std::abs(offsetShift) > mFloatParams[CALIBRATION_MAX_OFFSET_SHIFT])
		{
			RCLCPP_WARN(this->get_logger(), "Calibration: can't move by offset that much at a time! (tried %f, max %f)",
						offsetShift,
						mFloatParams[CALIBRATION_MAX_OFFSET_SHIFT]);
			offsetShift = std::clamp(offsetShift, -mFloatParams[CALIBRATION_MAX_OFFSET_SHIFT], mFloatParams[CALIBRATION_MAX_OFFSET_SHIFT]);
		}

		modeSetPos(msg->vesc_id, startingPosition + offsetShift);
		break;

	case CalibrateMsg::ACTION_TYPE_SET_VELOCITY:
		if (!isRecordedStatusValid(msg->vesc_id))
		{
			RCLCPP_ERROR(this->get_logger(), "No recent motor status recorded - cannot verify starting position, won't rotate.");
			return;
		}

		float precise_pos = mMotorStatuses[msg->vesc_id].position;
		// Don't allow to rotate the motor more than CALIBRATION_MAX_VELOCITY_SHIFT
		// away from origin.
		// If you want to move further, you have to set origin and repeat
		if (
			std::abs(precise_pos) > mFloatParams[CALIBRATION_MAX_VELOCITY_SHIFT] &&
			signum(msg->value) == signum(precise_pos) // Trying to move away from origin
		)
		{
			RCLCPP_WARN_THROTTLE(
				this->get_logger(), *this->get_clock(), 5 * 1000,
				"Trying to move too far at once. To rotate more, set origin and try again.");
			if (mMode == Mode::SetVelocity)
				modeNothing();
			return;
		}

		// Limit value
		float velocity = msg->value;
		if (std::abs(velocity) > mFloatParams[CALIBRATION_MAX_SPEED])
		{
			RCLCPP_WARN(this->get_logger(), "Calibration: provided a velocity that's too large (max %f)", mFloatParams[CALIBRATION_MAX_SPEED]);
			velocity = std::clamp(velocity, -mFloatParams[CALIBRATION_MAX_SPEED], mFloatParams[CALIBRATION_MAX_SPEED]);
		}

		// Velocity 0.0 is treated as a stop.
		if (velocity == 0.0f)
		{
			// If not holding yet (and hold not scheduled)
			if (mMode != Mode::Hold && !mHoldScheduled)
			{
				stopMotor(msg->vesc_id); // Also cancels timeout
				if (mIntParams[CALIBRATION_USE_SCHEDULE_HOLD])
				{
					modeNothing();
					scheduleHold(msg->vesc_id);
				}
				else
					modeHold(msg->vesc_id);
			}
			// Already holding, do nothing
		}
		else
		{
			modeSetVelocity(msg->vesc_id, velocity);
			startTimeout();
		}

		break;
	}
}

void CalibrateAxis::handleRoverStatus(const rex_interfaces::msg::RoverStatus::ConstSharedPtr &msg)
{
	if (mLastRoverStatus &&
		mLastRoverStatus->control_mode == rex_interfaces::msg::RoverStatus::CONTROL_MODE_ESTOP &&
		msg->control_mode != rex_interfaces::msg::RoverStatus::CONTROL_MODE_ESTOP)
	{
		if (mMode == Mode::SetPos || mMode == Mode::SetVelocity)
			stopMotor(mCurrentMotorID);
		modeNothing();
	}
	mLastRoverStatus = msg;
}

void CalibrateAxis::handleBatteryInfo(const rex_interfaces::msg::BatteryInfo::ConstSharedPtr &msg)
{
	if (msg->hotswap_status & rex_interfaces::msg::BatteryInfo::DRIVE_STOP)
	{
		if (!mLastBatteryInfo ||
			!(mLastBatteryInfo->hotswap_status & rex_interfaces::msg::BatteryInfo::DRIVE_STOP))
		{
			modeNothing();
		}
	}
	mLastBatteryInfo = msg;
}

// ######################### MODES #########################

void CalibrateAxis::modeNothing()
{
	RCLCPP_INFO(this->get_logger(), "MODE Nothing");
	mHoldScheduled = false;
	mFrameToSend = rex_interfaces::msg::VescMotorCommand();
	mMode = Mode::Nothing;
	mCurrentMotorID = 0;
	cancelTimeout();
}

void CalibrateAxis::modeSetPos(VESC_Id_t vescID, float pos)
{
	RCLCPP_INFO(this->get_logger(), "MODE SetPos [%d]: %f", vescID, pos);
	mHoldScheduled = false;
	mFrameToSend = frameSetPosition(vescID, pos);
	mMode = Mode::SetPos;
	mCurrentMotorID = vescID;
}

void CalibrateAxis::modeSetVelocity(VESC_Id_t vescID, float velocity)
{
	RCLCPP_INFO(this->get_logger(), "MODE SetVelocity [%d]: %f", vescID, velocity);
	mHoldScheduled = false;
	mFrameToSend = frameSetVelocity(vescID, velocity);
	mMode = Mode::SetVelocity;
	mCurrentMotorID = vescID;
}

void CalibrateAxis::modeHold(VESC_Id_t vescID)
{
	mHoldScheduled = false;
	if (mMode == Mode::SetPos)
	{
		// Keep sending the same frame
		mMode = Mode::Hold;
	}
	else if (!isRecordedStatusValid(vescID))
	{
		RCLCPP_ERROR(this->get_logger(), "Tried to hold, but position is outdated");
		modeNothing();
		return;
	}
	else
	{
		mFrameToSend = frameSetPosition(vescID, mMotorStatuses[vescID].position);
		mMode = Mode::Hold;
	}
	RCLCPP_INFO(this->get_logger(), "MODE Hold [%d]: %f", vescID, mFrameToSend.set_value * 100.0);
	mCurrentMotorID = vescID;
}

// ######################### TIMER-RELATED #########################

void CalibrateAxis::startTimeout()
{
	mVelocityTimeoutTimer->reset();
}

void CalibrateAxis::cancelTimeout()
{
	mVelocityTimeoutTimer->cancel();
}

void CalibrateAxis::timeoutTimerTick()
{
	VESC_Id_t vescID = mCurrentMotorID;
	if (!vescID || !calibrationMotorsContains(vescID))
	{
		RCLCPP_ERROR(this->get_logger(), "Timeout handler: Invalid motor ID");
		cancelTimeout();
		return;
	}
	if (mMode != Mode::SetVelocity)
	{
		RCLCPP_ERROR(this->get_logger(), "Error: timeout timer wasn't cancelled correctly!");
		cancelTimeout();
		return;
	}
	RCLCPP_INFO(this->get_logger(), "Motor [%d] stopped by timeout", vescID);
	stopMotor(vescID);
	if (mIntParams[CALIBRATION_USE_SCHEDULE_HOLD])
	{
		modeNothing();
		scheduleHold(vescID);
	}
	else
		modeHold(vescID);
	cancelTimeout();
}

void CalibrateAxis::stopMotor(VESC_Id_t vescID)
{
	cancelTimeout();

	rex_interfaces::msg::VescMotorCommand fr = frameStop(vescID);
	mCalibrationMotorCommandPub->publish(fr);
}

// ######################### CAN FRAMES #########################

rex_interfaces::msg::VescMotorCommand CalibrateAxis::frameStop(VESC_Id_t vescID)
{
	rex_interfaces::msg::VescMotorCommand msg;

	msg.vesc_id = vescID;
	msg.command_id = VESC_COMMAND_SET_CURRENT;
	msg.set_value = 0.0f;

	msg.header.stamp = this->get_clock()->now();

	return msg;
}

rex_interfaces::msg::VescMotorCommand CalibrateAxis::frameSetOrigin(VESC_Id_t vescID)
{
	rex_interfaces::msg::VescMotorCommand msg;

	msg.vesc_id = vescID;
	msg.command_id = VESC_COMMAND_SET_ORIGIN;
	msg.set_origin_data = 0.0f;

	msg.header.stamp = this->get_clock()->now();

	return msg;
}

rex_interfaces::msg::VescMotorCommand CalibrateAxis::frameSetPosition(VESC_Id_t vescID, float position)
{
	rex_interfaces::msg::VescMotorCommand msg;

	msg.vesc_id = vescID;
	msg.command_id = VESC_COMMAND_SET_POS;

	// Scale related to https://github.com/AlvaroBajceps/libVescCan/issues/10
	msg.set_value = position / 100.0f;

	msg.header.stamp = this->get_clock()->now();

	return msg;
}

rex_interfaces::msg::VescMotorCommand CalibrateAxis::frameSetVelocity(VESC_Id_t vescID, float velocity)
{
	rex_interfaces::msg::VescMotorCommand msg;

	msg.vesc_id = vescID;
	msg.command_id = VESC_COMMAND_SET_RPM;
	msg.set_value = velocity;

	msg.header.stamp = this->get_clock()->now();

	return msg;
}

void CalibrateAxis::sendFrame()
{
	if (!mLastRoverStatus || mLastRoverStatus->control_mode != rex_interfaces::msg::RoverStatus::CONTROL_MODE_ESTOP)
		return;
	if (mMode != Mode::Nothing)
	{
		mFrameToSend.header.stamp = this->get_clock()->now();
		mCalibrationMotorCommandPub->publish(mFrameToSend);
	}
}

// ######################### UTILITY #########################

bool CalibrateAxis::calibrationMotorsContains(VESC_Id_t vescID)
{
	// Check if the supplied ID is in the list of motors allowed for calibration.
	return std::find(mCalibrationMotors.begin(), mCalibrationMotors.end(), vescID) != mCalibrationMotors.end();
}

bool CalibrateAxis::checkSetPosEndCondition(const rex_interfaces::msg::VescStatus::ConstSharedPtr &msg)
{
	// Checks if SetPos mode is ready to finished (wheel is at target position)
	// Only run during Mode::SetPos!
	if (mMode != Mode::SetPos)
		return false;

	// Scale related to https://github.com/AlvaroBajceps/libVescCan/issues/10
	float targetValue = mFrameToSend.set_value * 100.0;

	if (mIntParams[CALIBRATION_LOG_SETPOS_DIFF])
		RCLCPP_INFO(
			this->get_logger(), "Target is %f, current is %f, diff is %f, tolerance is %f",
			targetValue, msg->precise_pos, std::abs(targetValue - msg->precise_pos), mFloatParams[CALIBRATION_STOP_TOLERANCE]);

	return msg->erpm == 0 && std::abs(msg->precise_pos - targetValue) <= mFloatParams[CALIBRATION_STOP_TOLERANCE];
}

bool CalibrateAxis::isTimestampOutdated(rclcpp::Time stamp)
{
	rclcpp::Time now = this->get_clock()->now();
	return (now - stamp).seconds() > mFloatParams[CALIBRATION_OUTDATED_DURATION_S];
}

bool CalibrateAxis::isRecordedStatusValid(VESC_Id_t vescID)
{
	// Checks if motor status is missing or outdated

	if (!mMotorStatuses.count(vescID))
	{
		return false;
	}

	if (isTimestampOutdated(mMotorStatuses[vescID].receivedAt))
	{
		return false;
	}
	return true;
}

void CalibrateAxis::scheduleHold(VESC_Id_t vescID)
{
	mHoldScheduled = true;
	mCurrentMotorID = vescID;
}

template <typename T>
int signum(T val)
{
	if (val > 0)
		return 1;
	if (val < 0)
		return -1;
	return 0;
}
