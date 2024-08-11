#include "can_wrapper/VescMotorController.hpp"

VescMotorController::VescMotorController(ros::NodeHandle& nh)
{
	mStatusGrabber = nh.subscribe<can_msgs::Frame>(RosCanConstants::RosTopics::can_raw_RX,1024,&VescMotorController::statusGrabber,this);
	mStatusPublisher = nh.advertise<can_wrapper::VescStatus>(RosCanConstants::RosTopics::can_vesc_status,256);
}

void VescMotorController::statusGrabber(const can_msgs::Frame::ConstPtr &frame)
{
	if(!VescCan::CanFrame::isValidVescCanFrame(frame->id))
		return;

	auto vescFrame = VescCan::CanFrame(frame);
	auto key = MotorStatusKey(vescFrame.vescID, vescFrame.commandID);
	auto value = MotorStatusValue(vescFrame,frame->header.stamp);

	auto findResult = mMotorStatus.find(key);

	if( findResult == mMotorStatus.cend() )
	{
		mMotorStatus.insert(std::pair<MotorStatusKey,MotorStatusValue>(key,value));
	}
	else
	{
		if(key.commandId == VescCan::Consts::Command::STATUS_1)
			sendUpdate(key.vescId);
		mMotorStatus[key] = value;
	}
}

void VescMotorController::sendUpdate(uint8_t vescId)
{
	MotorStatusKey key;

	can_wrapper::VescStatus status;

	key = MotorStatusKey(vescId, VescCan::Consts::Command::STATUS_1);
	if(mMotorStatus.find(key) != mMotorStatus.cend())
	{
		status.ERPM = VescCan::ConverterI32<VescCan::Consts::STATUS_1_ERPM_SCALE>::decode(mMotorStatus[key].vescFrame.data.status1.eRpm);
		status.Current = VescCan::ConverterF16<VescCan::Consts::STATUS_1_CURRENT_SCALE>::decode(mMotorStatus[key].vescFrame.data.status1.current);
		status.DutyCycle = VescCan::ConverterF16<VescCan::Consts::STATUS_1_DUTYCYCLE_SCALE>::decode(mMotorStatus[key].vescFrame.data.status1.dutyCucle);
	}

	key = MotorStatusKey(vescId, VescCan::Consts::Command::STATUS_2);
	if(mMotorStatus.find(key) != mMotorStatus.cend())
	{
		status.AhUsed = VescCan::ConverterF32<VescCan::Consts::STATUS_2_AMPHOURS_SCALE>::decode(mMotorStatus[key].vescFrame.data.status2.ampHours);
		status.AhCharged = VescCan::ConverterF32<VescCan::Consts::STATUS_2_AMPHOURSCHG_SCALE>::decode(mMotorStatus[key].vescFrame.data.status2.ampHoursChg);
	}

	key = MotorStatusKey(vescId, VescCan::Consts::Command::STATUS_3);
	if(mMotorStatus.find(key) != mMotorStatus.cend())
	{
		status.WhUsed = VescCan::ConverterF32<VescCan::Consts::STATUS_3_WATTHOURS_SCALE>::decode(mMotorStatus[key].vescFrame.data.status3.wattHours);
		status.WhCharged = VescCan::ConverterF32<VescCan::Consts::STATUS_3_WATTHOURSCHG_SCALE>::decode(mMotorStatus[key].vescFrame.data.status3.wattHoursChg);
	}

	key = MotorStatusKey(vescId, VescCan::Consts::Command::STATUS_4);
	if(mMotorStatus.find(key) != mMotorStatus.cend())
	{
		status.TempFet = VescCan::ConverterF16<VescCan::Consts::STATUS_4_TEMPFET_SCALE>::decode(mMotorStatus[key].vescFrame.data.status4.tempFet);
		status.TempMotor = VescCan::ConverterF16<VescCan::Consts::STATUS_4_TEMPMOTOR_SCALE>::decode(mMotorStatus[key].vescFrame.data.status4.tempMotor);
		status.CurrentIn = VescCan::ConverterF16<VescCan::Consts::STATUS_4_CURRENTIN_SCALE>::decode(mMotorStatus[key].vescFrame.data.status4.currentIn);
		status.PidPos = VescCan::ConverterF16<VescCan::Consts::STATUS_4_PIDPOS_SCALE>::decode(mMotorStatus[key].vescFrame.data.status4.pidPos);
	}

	key = MotorStatusKey(vescId, VescCan::Consts::Command::STATUS_5);
	if(mMotorStatus.find(key) != mMotorStatus.cend())
	{
		status.Tachometer = VescCan::ConverterF32<VescCan::Consts::STATUS_5_TACHOMETER_SCALE>::decode(mMotorStatus[key].vescFrame.data.status5.tachometer);
		status.VoltsIn = VescCan::ConverterF16<VescCan::Consts::STATUS_5_VOLTSIN_SCALE>::decode(mMotorStatus[key].vescFrame.data.status5.voltsIn);
	}

	key = MotorStatusKey(vescId, VescCan::Consts::Command::STATUS_6);
	if(mMotorStatus.find(key) != mMotorStatus.cend())
	{
		status.ADC1 = VescCan::ConverterF16<VescCan::Consts::STATUS_6_ADC1_SCALE>::decode(mMotorStatus[key].vescFrame.data.status6.adc1);
		status.ADC2 = VescCan::ConverterF16<VescCan::Consts::STATUS_6_ADC2_SCALE>::decode(mMotorStatus[key].vescFrame.data.status6.adc2);
		status.ADC3 = VescCan::ConverterF16<VescCan::Consts::STATUS_6_ADC3_SCALE>::decode(mMotorStatus[key].vescFrame.data.status6.adc3);
		status.PPM = VescCan::ConverterF16<VescCan::Consts::STATUS_6_PPM_SCALE>::decode(mMotorStatus[key].vescFrame.data.status6.ppm);
	}

	key = MotorStatusKey(vescId, VescCan::Consts::Command::STATUS_7);
	if(mMotorStatus.find(key) != mMotorStatus.cend())
	{
		status.PrecisePos = VescCan::ConverterF64<VescCan::Consts::STATUS_7_PRECISE_POS>::decode(mMotorStatus[key].vescFrame.data.status7.precisePos);
	}

	status.VescId = key.vescId;
	status.header.stamp = ros::Time::now();

	ROS_INFO("Published status!");
	lastSendTime = ros::Time::now();
	mStatusPublisher.publish(status);
}

void VescMotorController::clear()
{
	mMotorStatus.clear();
}