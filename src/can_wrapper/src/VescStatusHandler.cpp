#include "can_wrapper/VescStatusHandler.hpp"

VescStatusHandler::VescStatusHandler(ros::NodeHandle& nh)
{
	mStatusGrabber = nh.subscribe<can_msgs::Frame>(RosCanConstants::RosTopics::can_raw_RX,1024,&VescStatusHandler::statusGrabber,this);
	mStatusPublisher = nh.advertise<can_wrapper::VescStatus>(RosCanConstants::RosTopics::can_vesc_status,256);
}

void VescStatusHandler::statusGrabber(const can_msgs::Frame::ConstPtr &frame)
{
	auto vescFrame = VescInterop::rosToVesc(*frame);
	auto key = MotorStatusKey(vescFrame.vescID, (VESC_Command)vescFrame.command);
	auto value = MotorStatusValue(vescFrame,frame->header.stamp);

	auto findResult = mMotorStatus.find(key);

	if( findResult == mMotorStatus.cend() )
	{
		mMotorStatus.insert(std::pair<MotorStatusKey,MotorStatusValue>(key,value));
	}
	else
	{
		if(key.commandId == VESC_COMMAND_STATUS_1)
			sendUpdate(key.vescId);
		mMotorStatus[key] = value;
	}
}

void VescStatusHandler::sendUpdate(uint8_t vescId)
{
	MotorStatusKey key;

	can_wrapper::VescStatus status;

	key = MotorStatusKey(vescId, VESC_COMMAND_STATUS_1);
	if(mMotorStatus.find(key) != mMotorStatus.cend())
	{
		VESC_Status_1 statusData;
		VESC_ZeroMemory(&statusData, sizeof(statusData));
		VESC_convertRawToStatus1(&statusData, &mMotorStatus[key].vescFrame);

		status.ERPM = statusData.erpm;
		status.Current = statusData.current;
		status.DutyCycle = statusData.dutyCycle;
	}

	key = MotorStatusKey(vescId, VESC_COMMAND_STATUS_2);
	if(mMotorStatus.find(key) != mMotorStatus.cend())
	{
		VESC_Status_2 statusData;
		VESC_ZeroMemory(&statusData, sizeof(statusData));
		VESC_convertRawToStatus2(&statusData, &mMotorStatus[key].vescFrame);

		status.AhUsed = statusData.apmHours;
		status.AhCharged = statusData.apmHoursChg;
	}

	key = MotorStatusKey(vescId, VESC_COMMAND_STATUS_3);
	if(mMotorStatus.find(key) != mMotorStatus.cend())
	{
		VESC_Status_3 statusData;
		VESC_ZeroMemory(&statusData, sizeof(statusData));
		VESC_convertRawToStatus3(&statusData, &mMotorStatus[key].vescFrame);

		status.WhUsed = statusData.wattHours;
		status.WhCharged = statusData.wattHoursChg;
	}

	key = MotorStatusKey(vescId, VESC_COMMAND_STATUS_4);
	if(mMotorStatus.find(key) != mMotorStatus.cend())
	{
		VESC_Status_4 statusData;
		VESC_ZeroMemory(&statusData, sizeof(statusData));
		VESC_convertRawToStatus4(&statusData, &mMotorStatus[key].vescFrame);

		status.TempFet = statusData.tempFet;
		status.TempMotor = statusData.tempMotor;
		status.CurrentIn = statusData.currentIn;
		status.PidPos = statusData.pidPos;
	}

	key = MotorStatusKey(vescId, VESC_COMMAND_STATUS_5);
	if(mMotorStatus.find(key) != mMotorStatus.cend())
	{
		VESC_Status_5 statusData;
		VESC_ZeroMemory(&statusData, sizeof(statusData));
		VESC_convertRawToStatus5(&statusData, &mMotorStatus[key].vescFrame);

		status.Tachometer = statusData.tachometer;
		status.VoltsIn = statusData.voltsIn;
	}

	key = MotorStatusKey(vescId, VESC_COMMAND_STATUS_6);
	if(mMotorStatus.find(key) != mMotorStatus.cend())
	{
		VESC_Status_6 statusData;
		VESC_ZeroMemory(&statusData, sizeof(statusData));
		VESC_convertRawToStatus6(&statusData, &mMotorStatus[key].vescFrame);

		status.ADC1 = statusData.adc1;
		status.ADC2 = statusData.adc2;
		status.ADC3 = statusData.adc3;
		status.PPM = statusData.ppm; 	
	}

	key = MotorStatusKey(vescId, VESC_COMMAND_STATUS_7);
	if(mMotorStatus.find(key) != mMotorStatus.cend())
	{
		VESC_Status_7 statusData;
		VESC_ZeroMemory(&statusData, sizeof(statusData));
		VESC_convertRawToStatus7(&statusData, &mMotorStatus[key].vescFrame);

		status.PrecisePos = statusData.precisePos;
	}

	status.VescId = key.vescId;
	status.header.stamp = ros::Time::now();

	ROS_INFO("Published status!");
	lastSendTime = ros::Time::now();
	mStatusPublisher.publish(status);
}

void VescStatusHandler::clear()
{
	mMotorStatus.clear();
}