#include <can_bridge/SamplerControl.hpp>

SamplerControl::SamplerControl(const rclcpp::NodeOptions & options) : Node("sampler_control", options)
{
	const rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(256));

    mLastRoverStatus = std::make_shared<const RoverStatusMsg>();
    mLastBatteryInfo = std::make_shared<const BatteryInfoMsg>();

    mRawCanPub = this->create_publisher<CanFrame>(RosCanConstants::RosTopics::can_raw_TX, qos);

    mSamplerCtlSub = this->create_subscription<SamplerControlMsg>(
            RosCanConstants::RosTopics::mqtt_sampler_control, qos,
		    std::bind(&SamplerControl::handleSamplerCtl, this, std::placeholders::_1));

    mRoverStatusSub = this->create_subscription<RoverStatusMsg>(
		    RosCanConstants::RosTopics::mqtt_rover_status, qos,
		    std::bind(&SamplerControl::handleRoverStatus, this, std::placeholders::_1));

    mBatteryInfoSub = this->create_subscription<BatteryInfoMsg>(
            RosCanConstants::RosTopics::can_battery_info, qos,
            std::bind(&SamplerControl::handleBatteryInfo, this, std::placeholders::_1));

    mTimer = this->create_timer(std::chrono::milliseconds(50), std::bind(&SamplerControl::handleTimerClb, this));

	stopSampler();
}

void SamplerControl::handleSamplerCtl(const SamplerControlMsg::ConstSharedPtr &samplerCtlMsg)
{
    if (isSamplerMode(mLastRoverStatus))
        mLastSamplerCtl = samplerCtlMsg;
    else
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5 * 60 * 1000, // Throttle duration (5 minutes)
                             "When non-sampler mode is selected, incoming SamplerControl MQTT messages are discarded.");
}

void SamplerControl::handleRoverStatus(const RoverStatusMsg::ConstSharedPtr &roverStatusMsg)
{
    bool stop_sampler = false;
    if (!isSamplerMode(roverStatusMsg) && isSamplerMode(mLastRoverStatus))
        stop_sampler = true;

    if (isSamplerMode(roverStatusMsg) && !isSamplerMode(mLastRoverStatus))
        stop_sampler = true;

    mLastRoverStatus = roverStatusMsg;

    if (stop_sampler)
        stopSampler();
}

void SamplerControl::handleBatteryInfo(const BatteryInfoMsg::ConstSharedPtr &msg)
{
    mLastBatteryInfo = msg;
}

bool SamplerControl::isSamplerMode(const RoverStatusMsg::ConstSharedPtr &msg)
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

    // DEEP_SAMPLER, SURFACE_SAMPLER, DEEP_SAMPLER_AUTONOMY, SURFACE_SAMPLER_AUTONOMY
    return mode & (RoverStatusMsg::CONTROL_MODE_DEEP_SAMPLER |
            RoverStatusMsg::CONTROL_MODE_SURFACE_SAMPLER |
            RoverStatusMsg::CONTROL_MODE_DEEP_SAMPLER_AUTONOMY |
            RoverStatusMsg::CONTROL_MODE_SURFACE_SAMPLER_AUTONOMY);
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

    mLastSamplerCtl = std::make_shared<const SamplerControlMsg>(temp);

	mProbeDisableTimestamp = this->now();
}

void SamplerControl::handleTimerClb()
{
	bool is_within_grace_period = (this->now() - mProbeDisableTimestamp < rclcpp::Duration(1, 0)); // 1 sec grace

	if (isSamplerMode(mLastRoverStatus) || is_within_grace_period)
	{
		publishSamplerData();
	}
}

void SamplerControl::publishSamplerData()
{
	VESC_CommandFrame vesc_container[8];

	vesc_container[0].vescID = RosCanConstants::VescIds::sampler_platform;
	vesc_container[0].command = VESC_COMMAND_SET_DUTY;
	vesc_container[0].commandData = mLastSamplerCtl->platform_movement;

	vesc_container[1].vescID = RosCanConstants::VescIds::sampler_drill_mov;
	vesc_container[1].command = VESC_COMMAND_SET_DUTY;
	vesc_container[1].commandData = mLastSamplerCtl->drill_movement;

	vesc_container[2].vescID = RosCanConstants::VescIds::sampler_drill;
	vesc_container[2].command = VESC_COMMAND_SET_DUTY;
	vesc_container[2].commandData = mLastSamplerCtl->drill_action;

	vesc_container[3].vescID = RosCanConstants::VescIds::sampler_container_a;
	vesc_container[3].command = VESC_COMMAND_SET_POS;
	vesc_container[3].commandData = mLastSamplerCtl->container_degrees_a;

	vesc_container[4].vescID = RosCanConstants::VescIds::sampler_container_b;
	vesc_container[4].command = VESC_COMMAND_SET_POS;
	vesc_container[4].commandData = mLastSamplerCtl->container_degrees_b;

	vesc_container[5].vescID = RosCanConstants::VescIds::sampler_vacuum_suction;
	vesc_container[5].command = VESC_COMMAND_SET_DUTY;
	vesc_container[5].commandData = mLastSamplerCtl->vacuum_suction;

	vesc_container[6].vescID = RosCanConstants::VescIds::sampler_vacuum_a;
	vesc_container[6].command = VESC_COMMAND_SET_DUTY;
	vesc_container[6].commandData = mLastSamplerCtl->vacuum_a;

	vesc_container[7].vescID = RosCanConstants::VescIds::sampler_vacuum_b;
	vesc_container[7].command = VESC_COMMAND_SET_DUTY;
	vesc_container[7].commandData = mLastSamplerCtl->vacuum_b;

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

RCLCPP_COMPONENTS_REGISTER_NODE(SamplerControl)
