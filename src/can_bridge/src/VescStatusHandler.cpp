#include "can_bridge/VescStatusHandler.hpp"

VescStatusHandler::VescStatusHandler(const rclcpp::NodeOptions & options) : Node("Vesc_status_handler", options)
{
	const rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(256));

	mStatusGrabber = this->create_subscription<can_msgs::msg::Frame>(
		RosCanConstants::RosTopics::can_raw_RX, qos,
		std::bind(&VescStatusHandler::statusGrabber, this, std::placeholders::_1));

	mStatusPublisher = this->create_publisher<rex_interfaces::msg::VescStatus>(
		RosCanConstants::RosTopics::can_vesc_status, qos);

	mSendTimer = this->create_timer(std::chrono::milliseconds(6), std::bind(&VescStatusHandler::timer_method, this));

    mSetMotorVelSub = this->create_subscription<rex_interfaces::msg::Wheels>(
            RosCanConstants::RosTopics::can_set_motor_vel,
            qos, std::bind(&VescStatusHandler::handleSetMotorVel, this, std::placeholders::_1));
}

void VescStatusHandler::statusGrabber(const can_msgs::msg::Frame::ConstSharedPtr &frame)
{
	auto vescFrame = VescInterop::rosToVesc(*frame);

	//whitelist for bldc and cupamars
	switch(vescFrame.vescID)
	{
		//bldc-s
		case 0x50:
		case 0x51:
		case 0x52:
		case 0x53:
		//cupamars-es
		case 0x60:
		case 0x61:
		case 0x62:
		case 0x63:
			break;
		default:
			return;
	}

	switch(vescFrame.command)
	{
		case VESC_COMMAND_STATUS_1:
		case VESC_COMMAND_STATUS_2:
		case VESC_COMMAND_STATUS_3:
		case VESC_COMMAND_STATUS_4:
		case VESC_COMMAND_STATUS_5:
		case VESC_COMMAND_STATUS_6:
		case VESC_COMMAND_STATUS_7:
		case VESC_COMMAND_STATUS_11:
			break;
		default:
			return;
	}

    if (discoveredVescs.find(vescFrame.vescID) == discoveredVescs.end()) {
        discoveredVescs.insert(vescFrame.vescID);
        activeVescIDs.push_back(vescFrame.vescID);
        RCLCPP_INFO(this->get_logger(), "Discovered new VESC: 0x%02X", vescFrame.vescID);
    }

	auto key = MotorStatusKey(vescFrame.vescID, (VESC_Command)vescFrame.command);
	auto value = MotorStatusValue(vescFrame, frame->header.stamp);

	mMotorStatus[key] = value;

	mMotorLastUpdates[key.vescId] = this->now();
}

void VescStatusHandler::sendUpdate(uint8_t vescId)
{
	MotorStatusKey key;

	rex_interfaces::msg::VescStatus status;

	key = MotorStatusKey(vescId, VESC_COMMAND_STATUS_1);
	if (mMotorStatus.find(key) != mMotorStatus.cend())
	{
		VESC_Status_1 statusData;
		VESC_ZeroMemory(&statusData, sizeof(statusData));
		VESC_convertRawToStatus1(&statusData, &mMotorStatus[key].vescFrame);

		status.erpm = statusData.erpm;
		status.current = statusData.current;
		status.duty_cycle = statusData.dutyCycle;
	}

	key = MotorStatusKey(vescId, VESC_COMMAND_STATUS_2);
	if (mMotorStatus.find(key) != mMotorStatus.cend())
	{
		VESC_Status_2 statusData;
		VESC_ZeroMemory(&statusData, sizeof(statusData));
		VESC_convertRawToStatus2(&statusData, &mMotorStatus[key].vescFrame);

		status.ah_used = statusData.apmHours;
		status.ah_charged = statusData.apmHoursChg;
	}

	key = MotorStatusKey(vescId, VESC_COMMAND_STATUS_3);
	if (mMotorStatus.find(key) != mMotorStatus.cend())
	{
		VESC_Status_3 statusData;
		VESC_ZeroMemory(&statusData, sizeof(statusData));
		VESC_convertRawToStatus3(&statusData, &mMotorStatus[key].vescFrame);

		status.wh_used = statusData.wattHours;
		status.wh_charged = statusData.wattHoursChg;
	}

	key = MotorStatusKey(vescId, VESC_COMMAND_STATUS_4);
	if (mMotorStatus.find(key) != mMotorStatus.cend())
	{
		VESC_Status_4 statusData;
		VESC_ZeroMemory(&statusData, sizeof(statusData));
		VESC_convertRawToStatus4(&statusData, &mMotorStatus[key].vescFrame);

		status.temp_fet = statusData.tempFet;
		status.temp_motor = statusData.tempMotor;
		status.current_in = statusData.currentIn;
		status.pid_pos = statusData.pidPos;
	}

	key = MotorStatusKey(vescId, VESC_COMMAND_STATUS_5);
	if (mMotorStatus.find(key) != mMotorStatus.cend())
	{
		VESC_Status_5 statusData;
		VESC_ZeroMemory(&statusData, sizeof(statusData));
		VESC_convertRawToStatus5(&statusData, &mMotorStatus[key].vescFrame);

		status.tachometer = statusData.tachometer;
		status.volts_in = statusData.voltsIn;
	}

	key = MotorStatusKey(vescId, VESC_COMMAND_STATUS_6);
	if (mMotorStatus.find(key) != mMotorStatus.cend())
	{
		VESC_Status_6 statusData;
		VESC_ZeroMemory(&statusData, sizeof(statusData));
		VESC_convertRawToStatus6(&statusData, &mMotorStatus[key].vescFrame);

		status.adc1 = statusData.adc1;
		status.adc2 = statusData.adc2;
		status.adc3 = statusData.adc3;
		status.ppm = statusData.ppm;
	}

	key = MotorStatusKey(vescId, VESC_COMMAND_STATUS_7);
	if (mMotorStatus.find(key) != mMotorStatus.cend())
	{
		VESC_Status_7 statusData;
		VESC_ZeroMemory(&statusData, sizeof(statusData));
		VESC_convertRawToStatus7(&statusData, &mMotorStatus[key].vescFrame);

		status.precise_pos = statusData.precisePos;
	}

	key = MotorStatusKey(vescId, VESC_COMMAND_STATUS_11);
	if (mMotorStatus.find(key) != mMotorStatus.cend())
	{
		VESC_Status_11 statusData;
		VESC_ZeroMemory(&statusData, sizeof(statusData));
		VESC_convertRawToStatus11(&statusData, &mMotorStatus[key].vescFrame);

		rex_interfaces::msg::VescMotorCommand lastSentFrame;
		
		if(mLastSentFrame == nullptr)
			//placeholder in case no data is sent.
			lastSentFrame.command_id = VESC_COMMAND_STATUS_11;
		else if(key.vescId == RosCanConstants::VescIds::front_left_stepper)
			lastSentFrame = mLastSentFrame->front_left.turn;
		else if(key.vescId == RosCanConstants::VescIds::front_right_stepper)
			lastSentFrame = mLastSentFrame->front_right.turn;
		else if(key.vescId == RosCanConstants::VescIds::rear_right_stepper)
			lastSentFrame = mLastSentFrame->rear_right.turn;
		else if(key.vescId == RosCanConstants::VescIds::rear_left_stepper)
			lastSentFrame = mLastSentFrame->rear_left.turn;
		
		status.precise_pos = statusData.position;
		status.pid_pos = statusData.position;
		status.erpm = statusData.speed;
		status.current = statusData.current;
		status.temp_motor = statusData.motorTemp;

		if (lastSentFrame.command_id == VESC_COMMAND_SET_POS)
			status.pid_pos = lastSentFrame.set_value * 100; //libVescCan bug regarding Cubemars and SET_POS
		else if (lastSentFrame.command_id == VESC_COMMAND_SET_POS_SPEED_LOOP)
			status.pid_pos = lastSentFrame.set_pos_speed_loop_position;
		else if(lastSentFrame.command_id != VESC_COMMAND_STATUS_11)
			RCLCPP_WARN_ONCE(this->get_logger(), "Turn motor is not operated with SET_POS. Ghost wheel in RCA will be disabled.");
	}

	status.vesc_id = key.vescId;
	status.header.stamp = this->now();
	lastSendTime = status.header.stamp;
	mStatusPublisher->publish(status);
}

void VescStatusHandler::timer_method()
{
    if (activeVescIDs.empty()) return;

    if (currentVescIndex >= activeVescIDs.size()) {
        currentVescIndex = 0;
    }

    uint8_t vescId = activeVescIDs[currentVescIndex];

    auto last_update = mMotorLastUpdates[vescId];
    if (this->now() - last_update < rclcpp::Duration(1, 0)) {
        sendUpdate(vescId);
    } else {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "VESC 0x%02X is silent", vescId);
    }

    currentVescIndex++;
}

void VescStatusHandler::clear()
{
	mMotorStatus.clear();
}

void VescStatusHandler::handleSetMotorVel(const rex_interfaces::msg::Wheels::ConstSharedPtr &msg)
{
    mLastSentFrame = msg;
}

RCLCPP_COMPONENTS_REGISTER_NODE(VescStatusHandler)
