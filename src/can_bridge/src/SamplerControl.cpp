#include <can_bridge/SamplerControl.hpp>

SamplerControl::SamplerControl(rclcpp::Node::SharedPtr &nh) : mNh(nh)
{
	const rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(5));

	mRawCanPub = mNh->create_publisher<can_msgs::msg::Frame>(RosCanConstants::RosTopics::can_raw_TX, qos);
	mSamplerCtlSub = mNh->create_subscription<rex_interfaces::msg::SamplerControl>(
		RosCanConstants::RosTopics::mqtt_sampler_control, qos,
		std::bind(&SamplerControl::handleSamplerCtl, this, std::placeholders::_1));
	mTimerXd = mNh->create_timer(std::chrono::milliseconds(50), std::bind(&SamplerControl::doStuff, this));

	rex_interfaces::msg::SamplerControl temp;
	temp.container_degrees_a = 0;
	temp.container_degrees_b = 0;
	temp.drill_action = 0;
	temp.drill_movement = 0;
	temp.platform_movement = 0;

	mSamplerCtlMsgLast = std::make_shared<const rex_interfaces::msg::SamplerControl>(temp);
}

void SamplerControl::handleSamplerCtl(const rex_interfaces::msg::SamplerControl::ConstSharedPtr &probeCtlMsg)
{
	mSamplerCtlMsgLast = probeCtlMsg;
}

void SamplerControl::doStuff()
{
	VESC_CommandFrame vesc_container[3];

	// //note C0 servo is operated by sending value [0,179]
	// vesc_container[0].vescID = ID_C0;
	// vesc_container[0].command = VESC_COMMAND_SET_POS;
	// vesc_container[0].commandData = std::clamp(mSamplerCtlMsgLast->container_degrees_0, 0.0f, 179.0f);

	// //note C1 servo is operated by sending value [0,179]
	// vesc_container[1].vescID = ID_C1_C2;
	// vesc_container[1].command = VESC_COMMAND_SET_POS;
	// vesc_container[1].commandData = std::clamp(mSamplerCtlMsgLast->container_degrees_1, 0.0f, 179.0f);

	// //note C2 servo is operated by sending value [180,359]
	// vesc_container[2].vescID = ID_C1_C2;
	// vesc_container[2].command = VESC_COMMAND_SET_POS;
	// vesc_container[2].commandData = std::clamp(mSamplerCtlMsgLast->container_degrees_2, 0.0f, 179.0f) + 180.0f;

	vesc_container[0].vescID = RosCanConstants::VescIds::sampler_drill;
	vesc_container[0].command = VESC_COMMAND_SET_DUTY;
	vesc_container[0].commandData = mSamplerCtlMsgLast->drill_action;

	vesc_container[1].vescID = RosCanConstants::VescIds::sampler_drill_mov;
	vesc_container[1].command = VESC_COMMAND_SET_DUTY;
	vesc_container[1].commandData = mSamplerCtlMsgLast->drill_movement;

	vesc_container[2].vescID = RosCanConstants::VescIds::sampler_platform;
	vesc_container[2].command = VESC_COMMAND_SET_DUTY;
	vesc_container[2].commandData = mSamplerCtlMsgLast->platform_movement;

	publish(vesc_container, 3);
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