#include <can_bridge/SamplerControl.hpp>

SamplerControl::SamplerControl(rclcpp::Node::SharedPtr &nh) : mNh(nh)
{
	const rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(5));

	mRawCanPub = mNh->create_publisher<can_msgs::msg::Frame>(RosCanConstants::RosTopics::can_raw_TX, qos);
	mSamplerCtlSub = mNh->create_subscription<SamplerControlMsg>(
		RosCanConstants::RosTopics::mqtt_sampler_control, qos,
		std::bind(&SamplerControl::handleSamplerCtl, this, std::placeholders::_1));
	mRoverStatusSub = mNh->create_subscription<RoverStatusMsg>(
		RosCanConstants::RosTopics::mqtt_rover_status, qos,
		std::bind(&SamplerControl::handleRoverStatusClb, this, std::placeholders::_1));
	mTimer = mNh->create_timer(std::chrono::milliseconds(50), std::bind(&SamplerControl::handleTimerClb, this));

	mRoverStatusMsgLast = std::make_shared<const RoverStatusMsg>();

	stopSampler();
}

bool SamplerControl::isSamplerMode(const RoverStatusMsg::ConstSharedPtr &msg)
{
	return msg->communication_state == RoverStatusMsg::COMMUNICATION_STATE_OPENED &&
		   msg->control_mode == RoverStatusMsg::CONTROL_MODE_SAMPLER;
}

void SamplerControl::stopSampler()
{
	SamplerControlMsg temp;
	temp.container_degrees_a = 0;
	temp.container_degrees_b = 0;
	temp.drill_action = 0;
	temp.drill_movement = 0;
	temp.platform_movement = 0;
	temp.vacuum_a = 0;
	temp.vacuum_b = 0;
	temp.vacuum_suction = 0;

	mSamplerCtlMsgLast = std::make_shared<const SamplerControlMsg>(temp);

	mProbeDisableTimestamp = mNh->now();
}

void SamplerControl::handleSamplerCtl(const SamplerControlMsg::ConstSharedPtr &samplerCtlMsg)
{
	if (isSamplerMode(mRoverStatusMsgLast))
		mSamplerCtlMsgLast = samplerCtlMsg;
	else
		RCLCPP_WARN_THROTTLE(mNh->get_logger(), *mNh->get_clock(), 5 * 60 * 1000, // Throttle duration (5 minutes)
							 "When non-sampler mode is selected, incoming SamplerControl MQTT messages are discarded.");
}

void SamplerControl::handleRoverStatusClb(const RoverStatusMsg::ConstSharedPtr &roverStatusMsg)
{
	bool stop_sampler = false;
	if (!isSamplerMode(roverStatusMsg) && isSamplerMode(mRoverStatusMsgLast))
		stop_sampler = true;

	if (isSamplerMode(roverStatusMsg) && !isSamplerMode(mRoverStatusMsgLast))
		stop_sampler = true;

	mRoverStatusMsgLast = roverStatusMsg;

	if (stop_sampler)
		stopSampler();
}

void SamplerControl::handleTimerClb()
{
	bool is_within_grace_period = (mNh->now() - mProbeDisableTimestamp < rclcpp::Duration(1, 0)); // 1 sec grace

	if (isSamplerMode(mRoverStatusMsgLast) || is_within_grace_period)
	{
		publishSamplerData();
	}
}

void SamplerControl::publishSamplerData()
{
	VESC_CommandFrame vesc_container[8];

	vesc_container[0].vescID = RosCanConstants::VescIds::sampler_platform;
	vesc_container[0].command = VESC_COMMAND_SET_DUTY;
	vesc_container[0].commandData = mSamplerCtlMsgLast->platform_movement;

	vesc_container[1].vescID = RosCanConstants::VescIds::sampler_drill_mov;
	vesc_container[1].command = VESC_COMMAND_SET_DUTY;
	vesc_container[1].commandData = mSamplerCtlMsgLast->drill_movement;

	vesc_container[2].vescID = RosCanConstants::VescIds::sampler_drill;
	vesc_container[2].command = VESC_COMMAND_SET_DUTY;
	vesc_container[2].commandData = mSamplerCtlMsgLast->drill_action;

	vesc_container[3].vescID = RosCanConstants::VescIds::sampler_container_a;
	vesc_container[3].command = VESC_COMMAND_SET_POS;
	vesc_container[3].commandData = mSamplerCtlMsgLast->container_degrees_a;

	vesc_container[4].vescID = RosCanConstants::VescIds::sampler_container_b;
	vesc_container[4].command = VESC_COMMAND_SET_POS;
	vesc_container[4].commandData = mSamplerCtlMsgLast->container_degrees_b;

	vesc_container[5].vescID = RosCanConstants::VescIds::sampler_vacuum_suction;
	vesc_container[5].command = VESC_COMMAND_SET_DUTY;
	vesc_container[5].commandData = mSamplerCtlMsgLast->vacuum_suction;

	vesc_container[6].vescID = RosCanConstants::VescIds::sampler_vacuum_a;
	vesc_container[6].command = VESC_COMMAND_SET_DUTY;
	vesc_container[6].commandData = mSamplerCtlMsgLast->vacuum_a;

	vesc_container[7].vescID = RosCanConstants::VescIds::sampler_vacuum_b;
	vesc_container[7].command = VESC_COMMAND_SET_DUTY;
	vesc_container[7].commandData = mSamplerCtlMsgLast->vacuum_b;

	publish(vesc_container, 8);
}

void SamplerControl::publish(const VESC_CommandFrame *arr, int arr_size)
{

	VESC_RawFrame rf;
	for (int i = 0; i < arr_size; i++)
	{
		VESC_ZeroMemory(&rf, sizeof(VESC_RawFrame));
		VESC_convertCmdToRaw(&rf, &arr[i]);
		mRawCanPub->publish(VescInterop::vescToRos(rf));
	}
}