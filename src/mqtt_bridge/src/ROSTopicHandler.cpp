#include "mqtt_bridge/ROSTopicHandler.hpp"
#include <mqtt/async_client.h>
#include "rapidjson/writer.h"
#include "rapidjson/stringbuffer.h"
#include "rapidjson/error/en.h"

ROSTopicHandler::ROSTopicHandler(mqtt::async_client *mqttClient, int mqttQOS)
{
	ros::NodeHandle n;
	pub_Wheels = n.advertise<mqtt_bridge::Wheels>("/CAN/TX/set_motor_vel", 1000);
	sub_VescStatus = n.subscribe("/CAN/RX/vesc_status", 1000, &ROSTopicHandler::ROSTopicCallback_VescStatus, this);
	sub_ZedImuData = n.subscribe("/zed/zed_node/imu/data", 1000, &ROSTopicHandler::ROSTopicCallback_ZedImuData, this);
	cli = mqttClient;
	QOS = mqttQOS;
}

void ROSTopicHandler::publishMqttMessage(const std::string topicName, const char *message)
{
	mqtt::message_ptr pubmsg = mqtt::make_message(topicName, message);
	pubmsg->set_qos(QOS);
	cli->publish(pubmsg);
}

void ROSTopicHandler::addTimestampToJSON(rapidjson::Document &doc, long int nsec)
{
	rapidjson::Value k("Timestamp", doc.GetAllocator());
	rapidjson::Value v;
	v.SetUint64(nsec / 1000000);
	doc.AddMember(k, v, doc.GetAllocator());
}

template<typename T>
void ROSTopicHandler::addMemberToJSON(rapidjson::Document &doc, std::string name, T value)
{
    rapidjson::Value k(name, doc.GetAllocator());
	rapidjson::Value v(value);
	doc.AddMember(k, v, doc.GetAllocator());
}

template<typename T>
void ROSTopicHandler::addMembersFromMapToJSON(rapidjson::Document &doc, const std::map<std::string, T>& m)
{
	for (const auto& n : m) {
		addMemberToJSON(doc, n.first, n.second);
	}
}

void ROSTopicHandler::ROSTopicCallback_VescStatus(const mqtt_bridge::VescStatus::ConstPtr &receivedMsg)
{
	ROS_DEBUG("I received (ROS): a message (VescStatus)");

	rapidjson::Document d;
	d.SetObject();

	std::map<std::string, int> jsonIntFieldsMap{{"VescId", receivedMsg->VescId}, {"ERPM", receivedMsg->ERPM}};

	std::map<std::string, double> jsonDoubleFieldsMap{
	{"Current", receivedMsg->Current}, {"DutyCycle", receivedMsg->DutyCycle}, {"AhUsed", receivedMsg->AhUsed}, {"AhCharged", receivedMsg->AhCharged},
	{"WhUsed", receivedMsg->WhUsed}, {"WhCharged", receivedMsg->WhCharged}, {"TempFet", receivedMsg->TempFet}, {"TempMotor", receivedMsg->TempMotor},
	{"CurrentIn", receivedMsg->CurrentIn}, {"PidPos", receivedMsg->PidPos}, {"Tachometer", receivedMsg->Tachometer}, {"VoltsIn", receivedMsg->VoltsIn},
	{"ADC1", receivedMsg->ADC1}, {"ADC2", receivedMsg->ADC2}, {"ADC3", receivedMsg->ADC3}, {"PPM", receivedMsg->PPM}
	};

	addMembersFromMapToJSON(d, jsonIntFieldsMap);

	addMembersFromMapToJSON(d, jsonDoubleFieldsMap);

	addTimestampToJSON(d, receivedMsg->header.stamp.toNSec());

	rapidjson::StringBuffer buffer;
	rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
	d.Accept(writer);

	publishMqttMessage("RappTORS/VescStatus", buffer.GetString());

	ROS_DEBUG("I published (MQTT): [%s]", buffer.GetString());
}

void ROSTopicHandler::ROSTopicCallback_ZedImuData(const sensor_msgs::Imu::ConstPtr &receivedMsg)
{
	ROS_DEBUG("I received (ROS): a message (Imu)");

	rapidjson::Document d;
	d.SetObject();

	int jsonDoubleArray9Fields = 3;
	int jsonVector3Fields = 2;

	std::string jsonDoubleArray9FieldNames[jsonDoubleArray9Fields] = {"orientation_covariance", "angular_velocity_covariance", "linear_acceleration_covariance"};
	boost::array<double, 9> jsonDoubleArray9FieldValues[jsonDoubleArray9Fields] = {receivedMsg->orientation_covariance,
																				   receivedMsg->angular_velocity_covariance, receivedMsg->linear_acceleration_covariance};

	std::string jsonVector3FieldNames[jsonVector3Fields] = {"angular_velocity", "linear_acceleration"};
	geometry_msgs::Vector3 jsonVector3FieldValues[jsonVector3Fields] = {receivedMsg->angular_velocity, receivedMsg->linear_acceleration};

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
			coordinateFieldValue.SetDouble(receivedMsg->orientation.x);
			v.AddMember(coordinateFieldName, coordinateFieldValue, d.GetAllocator());
		}
		{
			rapidjson::Value coordinateFieldName("y", d.GetAllocator());
			rapidjson::Value coordinateFieldValue;
			coordinateFieldValue.SetDouble(receivedMsg->orientation.y);
			v.AddMember(coordinateFieldName, coordinateFieldValue, d.GetAllocator());
		}
		{
			rapidjson::Value coordinateFieldName("z", d.GetAllocator());
			rapidjson::Value coordinateFieldValue;
			coordinateFieldValue.SetDouble(receivedMsg->orientation.z);
			v.AddMember(coordinateFieldName, coordinateFieldValue, d.GetAllocator());
		}
		{
			rapidjson::Value coordinateFieldName("w", d.GetAllocator());
			rapidjson::Value coordinateFieldValue;
			coordinateFieldValue.SetDouble(receivedMsg->orientation.w);
			v.AddMember(coordinateFieldName, coordinateFieldValue, d.GetAllocator());
		}
		d.AddMember(k, v, d.GetAllocator());
	}

	this->addTimestampToJSON(d, receivedMsg->header.stamp.toNSec());

	rapidjson::StringBuffer buffer;
	rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
	d.Accept(writer);

	publishMqttMessage("RappTORS/ZedImuData", buffer.GetString());

	ROS_DEBUG("I published (MQTT): [%s]", buffer.GetString());
}

void ROSTopicHandler::publishMessage_Wheels(mqtt_bridge::Wheels message)
{
	pub_Wheels.publish(message);
	ROS_DEBUG("I published (ROS): a message (Wheels)");
}