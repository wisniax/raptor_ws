#include <can_bridge/ProbeControl.hpp>

ProbeControl::ProbeControl(rclcpp::Node::SharedPtr &nh) : mNh(nh)
{
	const rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(256));

	mRawCanPub = mNh->create_publisher<can_msgs::msg::Frame>(RosCanConstants::RosTopics::can_raw_TX, qos);
	mProbeCtlSub = mNh->create_subscription<can_bridge::msg::ProbeControl>(
		"/MQTT/SamplerControl", qos, 
		std::bind(&ProbeControl::handleProbeCtl, this, std::placeholders::_1)
	);
}

void ProbeControl::handleProbeCtl(const can_bridge::msg::ProbeControl::ConstSharedPtr& probeCtlMsg)
{
    VESC_CommandFrame vesc_container[3];

	//note C0 servo is operated by sending value [0,179]
	vesc_container[0].vescID = ID_C0;
	vesc_container[0].command = VESC_COMMAND_SET_POS;
	vesc_container[0].commandData = std::clamp(probeCtlMsg->container_degrees_0, 0.0f, 179.0f);

	//note C1 servo is operated by sending value [0,179]
	vesc_container[1].vescID = ID_C1_C2;
	vesc_container[1].command = VESC_COMMAND_SET_POS;
	vesc_container[1].commandData = std::clamp(probeCtlMsg->container_degrees_1, 0.0f, 179.0f);

	//note C2 servo is operated by sending value [180,359]
	vesc_container[2].vescID = ID_C1_C2;
	vesc_container[2].command = VESC_COMMAND_SET_POS;
	vesc_container[2].commandData = std::clamp(probeCtlMsg->container_degrees_2, 0.0f, 179.0f) + 180.0f;
	
	publish(vesc_container,3);

#warning Only for tests. Requesting status should be requestable from app.
	if(probeCtlMsg->drill_movement >= 1.0f)
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
	VESC_RawFrame rf;
	for(int i = 0; i < arr_size; i++)
	{
		VESC_ZeroMemory(&rf, sizeof(VESC_RawFrame));
		VESC_convertCmdToRaw(&rf, &arr[i]);
		mRawCanPub->publish(VescInterop::vescToRos(rf));
	}
}