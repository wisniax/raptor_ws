#include "mqtt_bridge/ROSTopicHandler.hpp"
#include <mqtt/async_client.h>
#include "rapidjson/writer.h"
#include "rapidjson/stringbuffer.h"
#include "rapidjson/error/en.h"

ROSTopicHandler::ROSTopicHandler(std::shared_ptr<mqtt::async_client> mqttClient, int mqttQOS)
{
	cli = mqttClient;
	QOS = mqttQOS;

	ros::NodeHandle n;

	sub_VescStatus = n.subscribe("/CAN/RX/vesc_status", 100, &ROSTopicHandler::ROSTopicCallback_VescStatus, this);
	timer_VescStatus = n.createTimer(ros::Duration(interval_VescStatus), &ROSTopicHandler::fire_VescStatus, this);

	msg_ZedImuData = std::make_shared<sensor_msgs::Imu>();
	sub_ZedImuData = n.subscribe("/zed/zed_node/imu/data", 100, &ROSTopicHandler::ROSTopicCallback_ZedImuData, this);
	timer_ZedImuData = n.createTimer(ros::Duration(interval_ZedImuData), &ROSTopicHandler::fire_ZedImuData, this);

	pub_Wheels = n.advertise<mqtt_bridge::Wheels>("/CAN/TX/set_motor_vel", 1000);
}

void ROSTopicHandler::publishMqttMessage(const std::string topicName, const char *message)
{
	ROS_DEBUG("Publishing MQTT message on topic [%s]: [%s]", topicName.c_str(), message);
	mqtt::message_ptr pubmsg = mqtt::make_message(topicName, message);
	pubmsg->set_qos(QOS);
	cli->publish(pubmsg);
}

// ###### json operations ######

void ROSTopicHandler::addTimestampToJSON(rapidjson::Document &doc, long int nsec)
{
	rapidjson::Value k("Timestamp", doc.GetAllocator());
	rapidjson::Value v;
	v.SetUint64(nsec / 1000000);
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

void ROSTopicHandler::fire_VescStatus(const ros::TimerEvent &event)
{
	//ROS_DEBUG("VescStatus timer fired");

	for (auto msgPair : msgMap_VescStatus)
	{
		PublishMqttMessage_VescStatus(msgPair.second);
	}
	
	msgMap_VescStatus.clear();
}

void ROSTopicHandler::ROSTopicCallback_VescStatus(const mqtt_bridge::VescStatus::ConstPtr &receivedMsg)
{
	//ROS_DEBUG("I received (ROS): a message (VescStatus)");

	std::map<int, std::shared_ptr<mqtt_bridge::VescStatus>>::iterator it = msgMap_VescStatus.find(receivedMsg->VescId);
	if (it != msgMap_VescStatus.end())
	{
		// if message for received VescId exists, then average the received msg with that message
		std::shared_ptr<mqtt_bridge::VescStatus> msg = it->second;
		msg->header.stamp = receivedMsg->header.stamp;
		msg->ERPM = (msg->ERPM + receivedMsg->ERPM) / 2;
		msg->Current = (msg->Current + receivedMsg->Current) / 2;
		msg->DutyCycle = (msg->DutyCycle + receivedMsg->DutyCycle) / 2;
		msg->AhUsed = (msg->AhUsed + receivedMsg->AhUsed) / 2;
		msg->AhCharged = (msg->AhCharged + receivedMsg->AhCharged) / 2;
		msg->WhUsed = (msg->WhUsed + receivedMsg->WhUsed) / 2;
		msg->WhCharged = (msg->WhCharged + receivedMsg->WhCharged) / 2;
		msg->TempFet = (msg->TempFet + receivedMsg->TempFet) / 2;
		msg->TempMotor = (msg->TempMotor + receivedMsg->TempMotor) / 2;
		msg->CurrentIn = (msg->CurrentIn + receivedMsg->CurrentIn) / 2;
		msg->PidPos = (msg->PidPos + receivedMsg->PidPos) / 2;
		msg->Tachometer = (msg->Tachometer + receivedMsg->Tachometer) / 2;
		msg->VoltsIn = (msg->VoltsIn + receivedMsg->VoltsIn) / 2;
		msg->ADC1 = (msg->ADC1 + receivedMsg->ADC1) / 2;
		msg->ADC2 = (msg->ADC2 + receivedMsg->ADC2) / 2;
		msg->ADC3 = (msg->ADC3 + receivedMsg->ADC3) / 2;
		msg->PPM = (msg->PPM + receivedMsg->PPM) / 2;
		return;
	}

	// if message for received VescId does not exist, then make a new message and insert it into the map
	std::shared_ptr<mqtt_bridge::VescStatus> msg = std::make_shared<mqtt_bridge::VescStatus>();

	msg->header.stamp = receivedMsg->header.stamp;
	msg->VescId = receivedMsg->VescId;
	msg->ERPM = receivedMsg->ERPM;
	msg->Current = receivedMsg->Current;
	msg->DutyCycle = receivedMsg->DutyCycle;
	msg->AhUsed = receivedMsg->AhUsed;
	msg->AhCharged = receivedMsg->AhCharged;
	msg->WhUsed = receivedMsg->WhUsed;
	msg->WhCharged = receivedMsg->WhCharged;
	msg->TempFet = receivedMsg->TempFet;
	msg->TempMotor = receivedMsg->TempMotor;
	msg->CurrentIn = receivedMsg->CurrentIn;
	msg->PidPos = receivedMsg->PidPos;
	msg->Tachometer = receivedMsg->Tachometer;
	msg->VoltsIn = receivedMsg->VoltsIn;
	msg->ADC1 = receivedMsg->ADC1;
	msg->ADC2 = receivedMsg->ADC2;
	msg->ADC3 = receivedMsg->ADC3;
	msg->PPM = receivedMsg->PPM;

	msgMap_VescStatus.insert({receivedMsg->VescId, msg});
}

void ROSTopicHandler::PublishMqttMessage_VescStatus(std::shared_ptr<mqtt_bridge::VescStatus> msg)
{
	rapidjson::Document d;
	d.SetObject();

	std::map<std::string, int> jsonIntFieldsMap{{"VescId", msg->VescId}, {"ERPM", msg->ERPM}};

	std::map<std::string, double> jsonDoubleFieldsMap{
		{"Current", msg->Current}, {"DutyCycle", msg->DutyCycle}, {"AhUsed", msg->AhUsed}, {"AhCharged", msg->AhCharged}, {"WhUsed", msg->WhUsed}, {"WhCharged", msg->WhCharged}, {"TempFet", msg->TempFet}, {"TempMotor", msg->TempMotor}, {"CurrentIn", msg->CurrentIn}, {"PidPos", msg->PidPos}, {"Tachometer", msg->Tachometer}, {"VoltsIn", msg->VoltsIn}, {"ADC1", msg->ADC1}, {"ADC2", msg->ADC2}, {"ADC3", msg->ADC3}, {"PPM", msg->PPM}};

	addMembersFromMapToJSON(d, jsonIntFieldsMap);

	addMembersFromMapToJSON(d, jsonDoubleFieldsMap);

	addTimestampToJSON(d, msg->header.stamp.toNSec());

	rapidjson::StringBuffer buffer;
	rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
	d.Accept(writer);

	publishMqttMessage("RappTORS/VescStatus", buffer.GetString());
}

// ###### ZedImuData ######

void ROSTopicHandler::fire_ZedImuData(const ros::TimerEvent &event)
{
	//ROS_DEBUG("Imu timer fired");
	if (!first_ZedImuData)
	{
		PublishMqttMessage_ZedImuData(msg_ZedImuData);
		first_ZedImuData = true;
	}
}

void ROSTopicHandler::ROSTopicCallback_ZedImuData(const sensor_msgs::Imu::ConstPtr &receivedMsg)
{
	//ROS_DEBUG("I received (ROS): a message (Imu)");

	if (!first_ZedImuData) {
		msg_ZedImuData->header.stamp = receivedMsg->header.stamp;
		msg_ZedImuData->orientation.x = (msg_ZedImuData->orientation.x + receivedMsg->orientation.x) / 2;
		msg_ZedImuData->orientation.y = (msg_ZedImuData->orientation.y + receivedMsg->orientation.y) / 2;
		msg_ZedImuData->orientation.z = (msg_ZedImuData->orientation.z + receivedMsg->orientation.z) / 2;
		msg_ZedImuData->orientation.w = (msg_ZedImuData->orientation.w + receivedMsg->orientation.w) / 2;

		msg_ZedImuData->angular_velocity.x = (msg_ZedImuData->angular_velocity.x + receivedMsg->angular_velocity.x) / 2;
		msg_ZedImuData->angular_velocity.y = (msg_ZedImuData->angular_velocity.y + receivedMsg->angular_velocity.y) / 2;
		msg_ZedImuData->angular_velocity.z = (msg_ZedImuData->angular_velocity.z + receivedMsg->angular_velocity.z) / 2;

		msg_ZedImuData->linear_acceleration.x = (msg_ZedImuData->linear_acceleration.x + receivedMsg->linear_acceleration.x) / 2;
		msg_ZedImuData->linear_acceleration.y = (msg_ZedImuData->linear_acceleration.y + receivedMsg->linear_acceleration.y) / 2;
		msg_ZedImuData->linear_acceleration.z = (msg_ZedImuData->linear_acceleration.z + receivedMsg->linear_acceleration.z) / 2;

		for (int i = 0; i < 9; i++)
		{
			msg_ZedImuData->orientation_covariance[i] = (msg_ZedImuData->orientation_covariance[i] + receivedMsg->orientation_covariance[i]) / 2;
			msg_ZedImuData->angular_velocity_covariance[i] = (msg_ZedImuData->angular_velocity_covariance[i] + receivedMsg->angular_velocity_covariance[i]) / 2;
			msg_ZedImuData->linear_acceleration_covariance[i] = (msg_ZedImuData->linear_acceleration_covariance[i] + receivedMsg->linear_acceleration_covariance[i]) / 2;
		}

		return;
	}

	msg_ZedImuData->header.stamp = receivedMsg->header.stamp;
	msg_ZedImuData->orientation = receivedMsg->orientation;
	msg_ZedImuData->angular_velocity = receivedMsg->angular_velocity;
	msg_ZedImuData->linear_acceleration = receivedMsg->linear_acceleration;
	msg_ZedImuData->orientation_covariance = receivedMsg->orientation_covariance;
	msg_ZedImuData->angular_velocity_covariance = receivedMsg->angular_velocity_covariance;
	msg_ZedImuData->linear_acceleration_covariance = receivedMsg->linear_acceleration_covariance;

	first_ZedImuData = false;
}

void ROSTopicHandler::PublishMqttMessage_ZedImuData(std::shared_ptr<sensor_msgs::Imu> msg)
{
	rapidjson::Document d;
	d.SetObject();

	int jsonDoubleArray9Fields = 3;
	int jsonVector3Fields = 2;

	std::string jsonDoubleArray9FieldNames[jsonDoubleArray9Fields] = {"orientation_covariance", "angular_velocity_covariance", "linear_acceleration_covariance"};
	boost::array<double, 9> jsonDoubleArray9FieldValues[jsonDoubleArray9Fields] = {msg->orientation_covariance,
																				   msg->angular_velocity_covariance, msg->linear_acceleration_covariance};

	std::string jsonVector3FieldNames[jsonVector3Fields] = {"angular_velocity", "linear_acceleration"};
	geometry_msgs::Vector3 jsonVector3FieldValues[jsonVector3Fields] = {msg->angular_velocity, msg->linear_acceleration};

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

	this->addTimestampToJSON(d, msg->header.stamp.toNSec());

	rapidjson::StringBuffer buffer;
	rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
	d.Accept(writer);

	publishMqttMessage("RappTORS/ZedImuData", buffer.GetString());
}

// ###### Wheels ######

void ROSTopicHandler::publishMessage_Wheels(mqtt_bridge::Wheels message)
{
	pub_Wheels.publish(message);
	ROS_DEBUG("I published (ROS): a message (Wheels)");
}