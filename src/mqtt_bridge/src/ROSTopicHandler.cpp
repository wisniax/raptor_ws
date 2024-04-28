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

	sub_VescStatus = n.subscribe("/CAN/RX/vesc_status", 5, &ROSTopicHandler::ROSTopicCallback_VescStatus, this);
	timer_VescStatus = n.createTimer(ros::Duration(1.0), &ROSTopicHandler::fire_VescStatus, this);

	sub_ZedImuData = n.subscribe("/zed/zed_node/imu/data", 5, &ROSTopicHandler::ROSTopicCallback_ZedImuData, this);

	pub_Wheels = n.advertise<mqtt_bridge::Wheels>("/CAN/TX/set_motor_vel", 1000);
	//ros::SubscribeOptions opts_VescStatus = ros::SubscribeOptions::create<mqtt_bridge::VescStatus>("/CAN/RX/vesc_status", 5, boost::bind(&ROSTopicHandler::ROSTopicCallback_VescStatus, this, _1), ros::VoidPtr(), queue_VescStatus);
	//sub_VescStatus = n.subscribe(opts_VescStatus);
}

ROSTopicHandler::~ROSTopicHandler()
{

}

void ROSTopicHandler::publishMqttMessage(const std::string topicName, const char *message)
{
	ROS_DEBUG("Publishing MQTT message on topic [%s]: [%s]", topicName.c_str(), message);
	mqtt::message_ptr pubmsg = mqtt::make_message(topicName, message);
	pubmsg->set_qos(QOS);
	//cli->publish(pubmsg);
}


// ###### json operations ######


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


// ###### VescStatus ######


void ROSTopicHandler::fire_VescStatus(const ros::TimerEvent& event)
{
	ROS_DEBUG("VescStatus timer fired");

	for (auto bufferPair : bufferMap_VescStatus) {
		VescStatusBufferPtr buffer = bufferPair.second;
		if (!buffer->empty()) {
			mqtt_bridge::VescStatus msg;

			// set timestamp from latest message
			msg.header.stamp = buffer->back()->header.stamp;

			// get oldest message from buffer and pop it
			const mqtt_bridge::VescStatus::ConstPtr firstMsg = buffer->front();
			buffer->pop_front();

			msg.VescId = bufferPair.first;
			msg.ERPM = firstMsg->ERPM;
			msg.Current = firstMsg->Current;
			msg.DutyCycle = firstMsg->DutyCycle;
			msg.AhUsed = firstMsg->AhUsed;
			msg.AhCharged = firstMsg->AhCharged;
			msg.WhUsed = firstMsg->WhUsed;
			msg.WhCharged = firstMsg->WhCharged;
			msg.TempFet = firstMsg->TempFet;
			msg.TempMotor = firstMsg->TempMotor;
			msg.CurrentIn = firstMsg->CurrentIn;
			msg.PidPos = firstMsg->PidPos;
			msg.Tachometer = firstMsg->Tachometer;
			msg.VoltsIn = firstMsg->VoltsIn;
			msg.ADC1 = firstMsg->ADC1;
			msg.ADC2 = firstMsg->ADC2;
			msg.ADC3 = firstMsg->ADC3;
			msg.PPM = firstMsg->PPM;

			for (const mqtt_bridge::VescStatus::ConstPtr receivedMsg : *buffer) {
				msg.ERPM = (msg.ERPM + receivedMsg->ERPM) / 2;
				msg.Current = (msg.Current + receivedMsg->Current) / 2;
				msg.DutyCycle = (msg.DutyCycle + receivedMsg->DutyCycle) / 2;
				msg.AhUsed = (msg.AhUsed + receivedMsg->AhUsed) / 2;
				msg.AhCharged = (msg.AhCharged + receivedMsg->AhCharged) / 2;
				msg.WhUsed = (msg.WhUsed + receivedMsg->WhUsed) / 2;
				msg.WhCharged = (msg.WhCharged + receivedMsg->WhCharged) / 2;
				msg.TempFet = (msg.TempFet + receivedMsg->TempFet) / 2;
				msg.TempMotor = (msg.TempMotor + receivedMsg->TempMotor) / 2;
				msg.CurrentIn = (msg.CurrentIn + receivedMsg->CurrentIn) / 2;
				msg.PidPos = (msg.PidPos + receivedMsg->PidPos) / 2;
				msg.Tachometer = (msg.Tachometer + receivedMsg->Tachometer) / 2;
				msg.VoltsIn = (msg.VoltsIn + receivedMsg->VoltsIn) / 2;
				msg.ADC1 = (msg.ADC1 + receivedMsg->ADC1) / 2;
				msg.ADC2 = (msg.ADC2 + receivedMsg->ADC2) / 2;
				msg.ADC3 = (msg.ADC3 + receivedMsg->ADC3) / 2;
				msg.PPM = (msg.PPM + receivedMsg->PPM) / 2;
			}

			buffer->clear();

			PublishMqttMessage_VescStatus(msg);
		} 
	}
}

void ROSTopicHandler::ROSTopicCallback_VescStatus(const mqtt_bridge::VescStatus::ConstPtr &receivedMsg)
{
	ROS_DEBUG("I received (ROS): a message (VescStatus)");

	std::map<int, VescStatusBufferPtr>::iterator it = bufferMap_VescStatus.find(receivedMsg->VescId);
	if (it != bufferMap_VescStatus.end())
	{
		// if buffer for received VescId exists, then push the received msg to that buffer
   		it->second->push_back(receivedMsg);
		return;
	}

	// if buffer for received VescId does not exist, then create a new buffer and push the received msg to that buffer
	VescStatusBufferPtr buffer = std::make_shared<boost::circular_buffer<mqtt_bridge::VescStatus::ConstPtr>>(5);
	buffer->push_back(receivedMsg);
	bufferMap_VescStatus.insert({receivedMsg->VescId, buffer});
}

void ROSTopicHandler::PublishMqttMessage_VescStatus(mqtt_bridge::VescStatus &msg) {
	rapidjson::Document d;
	d.SetObject();

	std::map<std::string, int> jsonIntFieldsMap{{"VescId", msg.VescId}, {"ERPM", msg.ERPM}};

	std::map<std::string, double> jsonDoubleFieldsMap{
	{"Current", msg.Current}, {"DutyCycle", msg.DutyCycle}, {"AhUsed", msg.AhUsed}, {"AhCharged", msg.AhCharged},
	{"WhUsed", msg.WhUsed}, {"WhCharged", msg.WhCharged}, {"TempFet", msg.TempFet}, {"TempMotor", msg.TempMotor},
	{"CurrentIn", msg.CurrentIn}, {"PidPos", msg.PidPos}, {"Tachometer", msg.Tachometer}, {"VoltsIn", msg.VoltsIn},
	{"ADC1", msg.ADC1}, {"ADC2", msg.ADC2}, {"ADC3", msg.ADC3}, {"PPM", msg.PPM}
	};

	addMembersFromMapToJSON(d, jsonIntFieldsMap);

	addMembersFromMapToJSON(d, jsonDoubleFieldsMap);

	addTimestampToJSON(d, msg.header.stamp.toNSec());

	rapidjson::StringBuffer buffer;
	rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
	d.Accept(writer);

	publishMqttMessage("RappTORS/VescStatus", buffer.GetString());
}


// ###### ZedImuData ######


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
}


// ###### Wheels ######


void ROSTopicHandler::publishMessage_Wheels(mqtt_bridge::Wheels message)
{
	pub_Wheels.publish(message);
	ROS_DEBUG("I published (ROS): a message (Wheels)");
}