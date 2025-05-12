#include <can_bridge/ProbeControl.hpp>

ProbeControl::ProbeControl(rclcpp::Node::SharedPtr &nh) : mNh(nh)
{
	const rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(256));

	mRawCanPub = mNh->create_publisher<can_msgs::msg::Frame>(RosCanConstants::RosTopics::can_raw_TX, qos);
	mProbeCtlSub = mNh->create_subscription<rex_interfaces::msg::ProbeControl>(
		"/MQTT/SamplerControl", qos, 
		std::bind(&ProbeControl::handleProbeCtl, this, std::placeholders::_1)
	);
	mTimerXd = mNh->create_timer(std::chrono::milliseconds(50), std::bind(&ProbeControl::doStuff, this));

	rex_interfaces::msg::ProbeControl temp;
	temp.container_degrees_0 = 0;
	temp.container_degrees_1 = 0;
	temp.container_degrees_2 = 0;
	temp.drill_action = 0;
	temp.drill_movement = 0;
	temp.platform_movement = 0;

	mProbeCtlMsgLast = std::make_shared<const rex_interfaces::msg::ProbeControl>(temp);
}

void ProbeControl::handleProbeCtl(const rex_interfaces::msg::ProbeControl::ConstSharedPtr& probeCtlMsg)
{
    mProbeCtlMsgLast = probeCtlMsg;
}

void ProbeControl::doStuff()
{
	VESC_CommandFrame vesc_container[6];

	//note C0 servo is operated by sending value [0,179]
	vesc_container[0].vescID = ID_C0;
	vesc_container[0].command = VESC_COMMAND_SET_POS;
	vesc_container[0].commandData = std::clamp(mProbeCtlMsgLast->container_degrees_0, 0.0f, 179.0f);

	//note C1 servo is operated by sending value [0,179]
	vesc_container[1].vescID = ID_C1_C2;
	vesc_container[1].command = VESC_COMMAND_SET_POS;
	vesc_container[1].commandData = std::clamp(mProbeCtlMsgLast->container_degrees_1, 0.0f, 179.0f);

	//note C2 servo is operated by sending value [180,359]
	vesc_container[2].vescID = ID_C1_C2;
	vesc_container[2].command = VESC_COMMAND_SET_POS;
	vesc_container[2].commandData = std::clamp(mProbeCtlMsgLast->container_degrees_2, 0.0f, 179.0f) + 180.0f;

	
	vesc_container[3].vescID = 0x82;
	vesc_container[3].command = VESC_COMMAND_SET_DUTY;
	vesc_container[3].commandData = mProbeCtlMsgLast->drill_action;
		
	vesc_container[4].vescID = 0x81;
	vesc_container[4].command = VESC_COMMAND_SET_DUTY;
	vesc_container[4].commandData = mProbeCtlMsgLast->drill_movement;
		
	vesc_container[5].vescID = 0x72;
	vesc_container[5].command = VESC_COMMAND_SET_DUTY;
	vesc_container[5].commandData = mProbeCtlMsgLast->platform_movement;

	publish(vesc_container,6);

#warning Only for tests. Requesting status should be requestable from app.
	if(mProbeCtlMsgLast->drill_movement >= 1.0f)
	{
		requestProbeStatus();
	}
}

void ProbeControl::requestProbeStatus()
{
	VESC_CommandFrame vesc_statusRq[2];

	vesc_statusRq[0].vescID = ID_C0;
	vesc_statusRq[0].command = VESC_COMMAND_SET_DUTY;
	vesc_statusRq[0].commandData = 0.1f;

	vesc_statusRq[1].vescID = ID_C1_C2;
	vesc_statusRq[1].command = VESC_COMMAND_SET_DUTY;
	vesc_statusRq[1].commandData = 0.1f;

	publish(vesc_statusRq,2);
}

void ProbeControl::publish(const VESC_CommandFrame* arr, int arr_size)
{
#warning ProbeControl publishnig disabled!
	return;

	VESC_RawFrame rf;
	for(int i = 0; i < arr_size; i++)
	{
		VESC_ZeroMemory(&rf, sizeof(VESC_RawFrame));
		VESC_convertCmdToRaw(&rf, &arr[i]);
		mRawCanPub->publish(VescInterop::vescToRos(rf));
	}
}