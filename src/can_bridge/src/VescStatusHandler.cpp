#include "can_bridge/VescStatusHandler.hpp"

VescStatusHandler::VescStatusHandler(rclcpp::Node::SharedPtr &nh) : mNh(nh)
{
	const rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(256));

	mStatusGrabber = nh->create_subscription<can_msgs::msg::Frame>(
		RosCanConstants::RosTopics::can_raw_RX, qos,
		std::bind(&VescStatusHandler::statusGrabber, this, std::placeholders::_1));

	mStatusPublisher = nh->create_publisher<rex_interfaces::msg::VescStatus>(
		RosCanConstants::RosTopics::can_vesc_status, qos);
}

void VescStatusHandler::statusGrabber(const can_msgs::msg::Frame::ConstSharedPtr &frame)
{
	auto vescFrame = VescInterop::rosToVesc(*frame);
	auto key = MotorStatusKey(vescFrame.vescID, (VESC_Command)vescFrame.command);
	auto value = MotorStatusValue(vescFrame, frame->header.stamp);

	auto findResult = mMotorStatus.find(key);

	if (findResult == mMotorStatus.cend())
	{
		mMotorStatus.insert(std::pair<MotorStatusKey, MotorStatusValue>(key, value));
	}
	else
	{
		if (key.commandId == VESC_COMMAND_STATUS_1)
			sendUpdate(key.vescId);
		mMotorStatus[key] = value;
	}
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

	status.vesc_id = key.vescId;
	status.header.stamp = rclcpp::Clock().now();
	RCLCPP_DEBUG(mNh->get_logger(), "Publishing status for VESC %d", key.vescId);
	lastSendTime = status.header.stamp;
	mStatusPublisher->publish(status);
}

void VescStatusHandler::clear()
{
	mMotorStatus.clear();
}