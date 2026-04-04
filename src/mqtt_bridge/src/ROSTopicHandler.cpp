#include "mqtt_bridge/ROSTopicHandler.hpp"
#include <mqtt/async_client.h>
#include "rapidjson/writer.h"
#include "rapidjson/stringbuffer.h"
#include "rapidjson/error/en.h"

ROSTopicHandler::ROSTopicHandler(std::shared_ptr<mqtt::async_client> mqttClient, int mqttQOS, rclcpp::Node::SharedPtr node)
{
	mCli = mqttClient;
	mQOS = mqttQOS;
	n = node;

	timer_cb_group = n->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

	mSub_VescStatus = n->create_subscription<rex_interfaces::msg::VescStatus>("/CAN/RX/vesc_status", 100, std::bind(&ROSTopicHandler::callback_VescStatus, this, std::placeholders::_1));
	mTimer_VescStatus = n->create_timer(std::chrono::milliseconds(mInterval_VescStatus), std::bind(&ROSTopicHandler::fire_VescStatus, this), timer_cb_group);

	mMsg_ZedImuData = std::make_shared<sensor_msgs::msg::Imu>();
	mSub_ZedImuData = n->create_subscription<sensor_msgs::msg::Imu>("/zed2/zed_node/imu/data", 100, std::bind(&ROSTopicHandler::callback_ZedImuData, this, std::placeholders::_1));
	mTimer_ZedImuData = n->create_timer(std::chrono::milliseconds(mInterval_ZedImuData), std::bind(&ROSTopicHandler::fire_ZedImuData, this), timer_cb_group);

	mSub_BatteryInfo = n->create_subscription<rex_interfaces::msg::BatteryInfo>("/CAN/RX/battery_info", 100, std::bind(&ROSTopicHandler::callback_BatteryInfo, this, std::placeholders::_1));
	mSub_SamplerFeedback = n->create_subscription<rex_interfaces::msg::SamplerFeedback>("/CAN/RX/sampler_status", 100, std::bind(&ROSTopicHandler::callback_SamplerFeedback, this, std::placeholders::_1));

	mPub_Wheels = n->create_publisher<rex_interfaces::msg::Wheels>("/CAN/TX/set_motor_vel", 1000);
	mPub_RoverControl = n->create_publisher<rex_interfaces::msg::RoverControl>("/MQTT/RoverControl", 1000);
	mPub_ManipulatorControl = n->create_publisher<rex_interfaces::msg::ManipulatorMqttMessage>("/MQTT/ManipulatorControl", 1000);
	mPub_SamplerControl = n->create_publisher<rex_interfaces::msg::SamplerControl>("/MQTT/SamplerControl", 1000);
	mPub_RoverStatus = n->create_publisher<rex_interfaces::msg::RoverStatus>("/MQTT/RoverStatus", 1000);
}

void ROSTopicHandler::publishMqttMessage(const std::string topicName, const char *message)
{
	RCLCPP_DEBUG_THROTTLE(n->get_logger(), *n->get_clock(), 500, "Publishing MQTT message on topic [%s]: [%s]", topicName.c_str(), message);
	mqtt::message_ptr pubmsg = mqtt::make_message(topicName, message);
	pubmsg->set_qos(mQOS);
	mCli->publish(pubmsg);
}

// ###### json operations ######

void ROSTopicHandler::addTimestampToJSON(rapidjson::Document &doc, long int msec)
{
	rapidjson::Value k("Timestamp", doc.GetAllocator());
	rapidjson::Value v;
	v.SetUint64(msec);
	doc.AddMember(k, v, doc.GetAllocator());
}

template <typename T>
void ROSTopicHandler::addMemberToJSON(rapidjson::Document &doc, std::string name, T value)
{
	rapidjson::Value k(name, doc.GetAllocator());
	rapidjson::Value v(value);
	doc.AddMember(k, v, doc.GetAllocator());
}

template <typename T>
void ROSTopicHandler::addMembersFromMapToJSON(rapidjson::Document &doc, const std::map<std::string, T> &m)
{
	for (const auto &n : m)
	{
		addMemberToJSON(doc, n.first, n.second);
	}
}

// ###### VescStatus ######

void ROSTopicHandler::fire_VescStatus()
{
	//RCLCPP_WARN(n->get_logger(), "VescStatus timer fired");
	for (auto msgPair : mMsgMap_VescStatus)
	{
		publishMqttMessage_VescStatus(msgPair.second);
	}
	
	mMsgMap_VescStatus.clear();
}

void ROSTopicHandler::callback_VescStatus(const rex_interfaces::msg::VescStatus::ConstSharedPtr &receivedMsg)
{
	std::map<int, std::shared_ptr<rex_interfaces::msg::VescStatus>>::iterator it = mMsgMap_VescStatus.find(receivedMsg->vesc_id);
	if (it != mMsgMap_VescStatus.end())
	{
		// if message for received vesc_id exists, then average the received msg with that message
		std::shared_ptr<rex_interfaces::msg::VescStatus> msg = it->second;
		msg->header.stamp = receivedMsg->header.stamp;
		msg->erpm = (msg->erpm + receivedMsg->erpm) / 2;
		msg->current = (msg->current + receivedMsg->current) / 2;
		msg->duty_cycle = (msg->duty_cycle + receivedMsg->duty_cycle) / 2;
		msg->ah_used = (msg->ah_used + receivedMsg->ah_used) / 2;
		msg->ah_charged = (msg->ah_charged + receivedMsg->ah_charged) / 2;
		msg->wh_used = (msg->wh_used + receivedMsg->wh_used) / 2;
		msg->wh_charged = (msg->wh_charged + receivedMsg->wh_charged) / 2;
		msg->temp_fet = (msg->temp_fet + receivedMsg->temp_fet) / 2;
		msg->temp_motor = (msg->temp_motor + receivedMsg->temp_motor) / 2;
		msg->current_in = (msg->current_in + receivedMsg->current_in) / 2;
		msg->pid_pos = (msg->pid_pos + receivedMsg->pid_pos) / 2;
		msg->tachometer = (msg->tachometer + receivedMsg->tachometer) / 2;
		msg->volts_in = (msg->volts_in + receivedMsg->volts_in) / 2;
		msg->adc1 = (msg->adc1 + receivedMsg->adc1) / 2;
		msg->adc2 = (msg->adc2 + receivedMsg->adc2) / 2;
		msg->adc3 = (msg->adc3 + receivedMsg->adc3) / 2;
		msg->ppm = (msg->ppm + receivedMsg->ppm) / 2;
		msg->precise_pos = (msg->precise_pos + receivedMsg->precise_pos) / 2;
		return;
	}

	// if message for received vesc_id does not exist, then make a new message and insert it into the map
	std::shared_ptr<rex_interfaces::msg::VescStatus> msg = std::make_shared<rex_interfaces::msg::VescStatus>();

	msg->header.stamp = receivedMsg->header.stamp;
	msg->vesc_id = receivedMsg->vesc_id;
	msg->erpm = receivedMsg->erpm;
	msg->current = receivedMsg->current;
	msg->duty_cycle = receivedMsg->duty_cycle;
	msg->ah_used = receivedMsg->ah_used;
	msg->ah_charged = receivedMsg->ah_charged;
	msg->wh_used = receivedMsg->wh_used;
	msg->wh_charged = receivedMsg->wh_charged;
	msg->temp_fet = receivedMsg->temp_fet;
	msg->temp_motor = receivedMsg->temp_motor;
	msg->current_in = receivedMsg->current_in;
	msg->pid_pos = receivedMsg->pid_pos;
	msg->tachometer = receivedMsg->tachometer;
	msg->volts_in = receivedMsg->volts_in;
	msg->adc1 = receivedMsg->adc1;
	msg->adc2 = receivedMsg->adc2;
	msg->adc3 = receivedMsg->adc3;
	msg->ppm = receivedMsg->ppm;
	msg->precise_pos = receivedMsg->precise_pos;

	mMsgMap_VescStatus.insert({receivedMsg->vesc_id, msg});
}

void ROSTopicHandler::publishMqttMessage_VescStatus(std::shared_ptr<rex_interfaces::msg::VescStatus> msg)
{
	rapidjson::Document d;
	d.SetObject();

	std::map<std::string, int> jsonIntFieldsMap{{"VescId", msg->vesc_id}, {"ERPM", msg->erpm}};

	std::map<std::string, double> jsonDoubleFieldsMap{
		{"Current", msg->current}, {"DutyCycle", msg->duty_cycle}, {"AhUsed", msg->ah_used}, {"AhCharged", msg->ah_charged}, {"WhUsed", msg->wh_used}, {"WhCharged", msg->wh_charged}, {"TempFet", msg->temp_fet}, {"TempMotor", msg->temp_motor}, {"CurrentIn", msg->current_in}, {"PidPos", msg->pid_pos}, {"Tachometer", msg->tachometer}, {"VoltsIn", msg->volts_in}, {"ADC1", msg->adc1}, {"ADC2", msg->adc2}, {"ADC3", msg->adc3}, {"PPM", msg->ppm}, {"PrecisePos", msg->precise_pos}};

	addMembersFromMapToJSON(d, jsonIntFieldsMap);

	addMembersFromMapToJSON(d, jsonDoubleFieldsMap);

	addTimestampToJSON(d, ((long int)msg->header.stamp.sec * 1000) + ((long int)msg->header.stamp.nanosec / 1000000));

	rapidjson::StringBuffer buffer;
	rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
	d.Accept(writer);

	publishMqttMessage("RappTORS/VescStatus", buffer.GetString());
}

// ###### ZedImuData ######

void ROSTopicHandler::fire_ZedImuData()
{
	if (!mFirst_ZedImuData)
	{
		publishMqttMessage_ZedImuData(mMsg_ZedImuData);
		mFirst_ZedImuData = true;
	}
}

void ROSTopicHandler::callback_ZedImuData(const sensor_msgs::msg::Imu::ConstSharedPtr &receivedMsg)
{
	if (!mFirst_ZedImuData) {
		mMsg_ZedImuData->header.stamp = receivedMsg->header.stamp;
		mMsg_ZedImuData->orientation.x = (mMsg_ZedImuData->orientation.x + receivedMsg->orientation.x) / 2;
		mMsg_ZedImuData->orientation.y = (mMsg_ZedImuData->orientation.y + receivedMsg->orientation.y) / 2;
		mMsg_ZedImuData->orientation.z = (mMsg_ZedImuData->orientation.z + receivedMsg->orientation.z) / 2;
		mMsg_ZedImuData->orientation.w = (mMsg_ZedImuData->orientation.w + receivedMsg->orientation.w) / 2;

		mMsg_ZedImuData->angular_velocity.x = (mMsg_ZedImuData->angular_velocity.x + receivedMsg->angular_velocity.x) / 2;
		mMsg_ZedImuData->angular_velocity.y = (mMsg_ZedImuData->angular_velocity.y + receivedMsg->angular_velocity.y) / 2;
		mMsg_ZedImuData->angular_velocity.z = (mMsg_ZedImuData->angular_velocity.z + receivedMsg->angular_velocity.z) / 2;

		mMsg_ZedImuData->linear_acceleration.x = (mMsg_ZedImuData->linear_acceleration.x + receivedMsg->linear_acceleration.x) / 2;
		mMsg_ZedImuData->linear_acceleration.y = (mMsg_ZedImuData->linear_acceleration.y + receivedMsg->linear_acceleration.y) / 2;
		mMsg_ZedImuData->linear_acceleration.z = (mMsg_ZedImuData->linear_acceleration.z + receivedMsg->linear_acceleration.z) / 2;

		for (int i = 0; i < 9; i++)
		{
			mMsg_ZedImuData->orientation_covariance[i] = (mMsg_ZedImuData->orientation_covariance[i] + receivedMsg->orientation_covariance[i]) / 2;
			mMsg_ZedImuData->angular_velocity_covariance[i] = (mMsg_ZedImuData->angular_velocity_covariance[i] + receivedMsg->angular_velocity_covariance[i]) / 2;
			mMsg_ZedImuData->linear_acceleration_covariance[i] = (mMsg_ZedImuData->linear_acceleration_covariance[i] + receivedMsg->linear_acceleration_covariance[i]) / 2;
		}

		return;
	}

	mMsg_ZedImuData->header.stamp = receivedMsg->header.stamp;
	mMsg_ZedImuData->orientation = receivedMsg->orientation;
	mMsg_ZedImuData->angular_velocity = receivedMsg->angular_velocity;
	mMsg_ZedImuData->linear_acceleration = receivedMsg->linear_acceleration;
	mMsg_ZedImuData->orientation_covariance = receivedMsg->orientation_covariance;
	mMsg_ZedImuData->angular_velocity_covariance = receivedMsg->angular_velocity_covariance;
	mMsg_ZedImuData->linear_acceleration_covariance = receivedMsg->linear_acceleration_covariance;

	mFirst_ZedImuData = false;
}

void ROSTopicHandler::publishMqttMessage_ZedImuData(std::shared_ptr<sensor_msgs::msg::Imu> msg)
{
	rapidjson::Document d;
	d.SetObject();

	int jsonDoubleArray9Fields = 3;
	int jsonVector3Fields = 2;

	std::string jsonDoubleArray9FieldNames[jsonDoubleArray9Fields] = {"orientation_covariance", "angular_velocity_covariance", "linear_acceleration_covariance"};
	std::array<double, 9> jsonDoubleArray9FieldValues[jsonDoubleArray9Fields] = {msg->orientation_covariance,
																				   msg->angular_velocity_covariance, msg->linear_acceleration_covariance};

	std::string jsonVector3FieldNames[jsonVector3Fields] = {"angular_velocity", "linear_acceleration"};
	geometry_msgs::msg::Vector3 jsonVector3FieldValues[jsonVector3Fields];
	jsonVector3FieldValues[0] = msg->angular_velocity;
	jsonVector3FieldValues[1] = msg->linear_acceleration;

	// float64[9]: orientation_covariance, angular_velocity_covariance, linear_acceleration_covariance
	for (int i = 0; i < jsonDoubleArray9Fields; i++)
	{
		rapidjson::Value k(jsonDoubleArray9FieldNames[i], d.GetAllocator());
		rapidjson::Value v;
		v.SetArray();
		for (int j = 0; j < 9; j++)
		{
			v.PushBack(jsonDoubleArray9FieldValues[i][j], d.GetAllocator());
		}
		d.AddMember(k, v, d.GetAllocator());
	}

	// geometry_msgs/Vector3: angular_velocity, linear_acceleration
	for (int i = 0; i < jsonVector3Fields; i++)
	{
		rapidjson::Value k(jsonVector3FieldNames[i], d.GetAllocator());
		rapidjson::Value v;
		v.SetObject();
		{

			rapidjson::Value coordinateFieldName("x", d.GetAllocator());
			rapidjson::Value coordinateFieldValue;
			coordinateFieldValue.SetDouble(jsonVector3FieldValues[i].x);
			v.AddMember(coordinateFieldName, coordinateFieldValue, d.GetAllocator());
		}
		{
			rapidjson::Value coordinateFieldName("y", d.GetAllocator());
			rapidjson::Value coordinateFieldValue;
			coordinateFieldValue.SetDouble(jsonVector3FieldValues[i].y);
			v.AddMember(coordinateFieldName, coordinateFieldValue, d.GetAllocator());
		}
		{
			rapidjson::Value coordinateFieldName("z", d.GetAllocator());
			rapidjson::Value coordinateFieldValue;
			coordinateFieldValue.SetDouble(jsonVector3FieldValues[i].z);
			v.AddMember(coordinateFieldName, coordinateFieldValue, d.GetAllocator());
		}
		d.AddMember(k, v, d.GetAllocator());
	}

	// geometry_msgs/Quaternion: orientation
	{
		rapidjson::Value k("orientation", d.GetAllocator());
		rapidjson::Value v;
		v.SetObject();
		{
			rapidjson::Value coordinateFieldName("x", d.GetAllocator());
			rapidjson::Value coordinateFieldValue;
			coordinateFieldValue.SetDouble(msg->orientation.x);
			v.AddMember(coordinateFieldName, coordinateFieldValue, d.GetAllocator());
		}
		{
			rapidjson::Value coordinateFieldName("y", d.GetAllocator());
			rapidjson::Value coordinateFieldValue;
			coordinateFieldValue.SetDouble(msg->orientation.y);
			v.AddMember(coordinateFieldName, coordinateFieldValue, d.GetAllocator());
		}
		{
			rapidjson::Value coordinateFieldName("z", d.GetAllocator());
			rapidjson::Value coordinateFieldValue;
			coordinateFieldValue.SetDouble(msg->orientation.z);
			v.AddMember(coordinateFieldName, coordinateFieldValue, d.GetAllocator());
		}
		{
			rapidjson::Value coordinateFieldName("w", d.GetAllocator());
			rapidjson::Value coordinateFieldValue;
			coordinateFieldValue.SetDouble(msg->orientation.w);
			v.AddMember(coordinateFieldName, coordinateFieldValue, d.GetAllocator());
		}
		d.AddMember(k, v, d.GetAllocator());
	}

	addTimestampToJSON(d, ((long int)msg->header.stamp.sec * 1000) + ((long int)msg->header.stamp.nanosec / 1000000));

	rapidjson::StringBuffer buffer;
	rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
	d.Accept(writer);

	publishMqttMessage("RappTORS/ZedImuData", buffer.GetString());
}

// ###### BatteryInfo ######
void ROSTopicHandler::callback_BatteryInfo(const rex_interfaces::msg::BatteryInfo::ConstSharedPtr &msg)
{
	rapidjson::Document d;
	d.SetObject();

	std::map<std::string, int> jsonIntFieldsMap{{"Slot", msg->slot}, {"ID", msg->id}, {"BatteryStatus", msg->battery_status}, {"HotswapStatus", msg->hotswap_status}};

	std::map<std::string, double> jsonDoubleFieldsMap{
		{"Voltage", msg->voltage}, {"Current", msg->current}, {"Temperature", msg->temperature}, {"ChargePercent", msg->charge_percent}};

		addMembersFromMapToJSON(d, jsonIntFieldsMap);

		addMembersFromMapToJSON(d, jsonDoubleFieldsMap);

		addTimestampToJSON(d, ((long int)msg->header.stamp.sec * 1000) + ((long int)msg->header.stamp.nanosec / 1000000));

		rapidjson::StringBuffer buffer;
		rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
		d.Accept(writer);

		publishMqttMessage("RappTORS/BatteryInfo", buffer.GetString());
}

// ###### SamplerFeedback ######
void ROSTopicHandler::callback_SamplerFeedback(const rex_interfaces::msg::SamplerFeedback::ConstSharedPtr &msg)
{
	rapidjson::Document d;
	d.SetObject();

	std::map<std::string, double> jsonDoubleFieldsMap{
		{"WeightA", msg->weight_a}, {"WeightB", msg->weight_b}, {"WeightC", msg->weight_c}, {"Ph", msg->ph}, {"Distance", msg->distance}};

		addMembersFromMapToJSON(d, jsonDoubleFieldsMap);

		addTimestampToJSON(d, ((long int)msg->header.stamp.sec * 1000) + ((long int)msg->header.stamp.nanosec / 1000000));

		rapidjson::StringBuffer buffer;
		rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
		d.Accept(writer);

		publishMqttMessage("RappTORS/SamplerFeedback", buffer.GetString());
}

// ###### Wheels ######

void ROSTopicHandler::publishMessage_Wheels(rex_interfaces::msg::Wheels message)
{
	mPub_Wheels->publish(message);
}

// ###### RoverControl ######

void ROSTopicHandler::publishMessage_RoverControl(rex_interfaces::msg::RoverControl message)
{
	mPub_RoverControl->publish(message);
}

// ##### ManipulatorControl ######

void ROSTopicHandler::publishMessage_ManipulatorControl(rex_interfaces::msg::ManipulatorMqttMessage message)
{
	mPub_ManipulatorControl->publish(message);
}

// ##### SamplerControl ######

void ROSTopicHandler::publishMessage_SamplerControl(rex_interfaces::msg::SamplerControl message)
{
	mPub_SamplerControl->publish(message);
}

// ##### RoverStatus ######

void ROSTopicHandler::publishMessage_RoverStatus(rex_interfaces::msg::RoverStatus message)
{
	mPub_RoverStatus->publish(message);
}
