#include "mqtt_bridge/ROSTopicHandler.hpp"
#include <mqtt/async_client.h>
#include "rapidjson/writer.h"
#include "rapidjson/stringbuffer.h"
#include "rapidjson/error/en.h"

ROSTopicHandler::ROSTopicHandler(mqtt::async_client *mqttClient, int mqttQOS)
{
	ros::NodeHandle n;
	pub = n.advertise<mqtt_bridge::Wheels>("/CAN/TX/set_motor_vel", 1000);
	sub = n.subscribe("/CAN/RX/vesc_status", 1000, &ROSTopicHandler::ROSTopicCallback, this);
	sub2 = n.subscribe("/zed/zed_node/imu/data", 1000, &ROSTopicHandler::ROSTopicCallback2, this);
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

void ROSTopicHandler::ROSTopicCallback(const mqtt_bridge::VescStatus::ConstPtr &receivedMsg)
{
	ROS_DEBUG("I received (ROS): a message (VescStatus)");

	rapidjson::Document d;
	d.SetObject();

	int jsonIntFields = 2;
	int jsonDoubleFields = 16;
	std::string jsonIntFieldNames[jsonIntFields] = {"VescId", "ERPM"};
	int jsonIntFieldValues[jsonIntFields] = {receivedMsg->VescId, receivedMsg->ERPM};
	std::string jsonDoubleFieldNames[jsonDoubleFields] = {"Current", "DutyCycle", "AhUsed", "AhCharged",
														  "WhUsed", "WhCharged", "TempFet", "TempMotor",
														  "CurrentIn", "PidPos", "Tachometer", "VoltsIn",
														  "ADC1", "ADC2", "ADC3", "PPM"};
	double jsonDoubleFieldValues[jsonDoubleFields] = {receivedMsg->Current, receivedMsg->DutyCycle, receivedMsg->AhUsed, receivedMsg->AhCharged,
													  receivedMsg->WhUsed, receivedMsg->WhCharged, receivedMsg->TempFet, receivedMsg->TempMotor,
													  receivedMsg->CurrentIn, receivedMsg->PidPos, receivedMsg->Tachometer, receivedMsg->VoltsIn,
													  receivedMsg->ADC1, receivedMsg->ADC2, receivedMsg->ADC3, receivedMsg->PPM};

	for (int i = 0; i < jsonIntFields; i++)
	{
		rapidjson::Value k(jsonIntFieldNames[i], d.GetAllocator());
		rapidjson::Value v;
		v.SetInt(jsonIntFieldValues[i]);
		d.AddMember(k, v, d.GetAllocator());
	}

	for (int i = 0; i < jsonDoubleFields; i++)
	{
		rapidjson::Value k(jsonDoubleFieldNames[i], d.GetAllocator());
		rapidjson::Value v;
		v.SetDouble(jsonDoubleFieldValues[i]);
		d.AddMember(k, v, d.GetAllocator());
	}

	this->addTimestampToJSON(d, receivedMsg->header.stamp.toNSec());

	rapidjson::StringBuffer buffer;
	rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
	d.Accept(writer);

	publishMqttMessage("RappTORS/VescStatus", buffer.GetString());

	ROS_DEBUG("I published (MQTT): [%s]", buffer.GetString());
}

void ROSTopicHandler::ROSTopicCallback2(const sensor_msgs::Imu::ConstPtr &receivedMsg)
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

	// orientation
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

void ROSTopicHandler::publishMessage(mqtt_bridge::Wheels message)
{
	pub.publish(message);
	ROS_DEBUG("I published (ROS): a message (Wheels)");
}