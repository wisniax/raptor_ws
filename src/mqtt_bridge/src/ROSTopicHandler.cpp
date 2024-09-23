#include "mqtt_bridge/ROSTopicHandler.hpp"
#include <mqtt/async_client.h>
#include "rapidjson/writer.h"
#include "rapidjson/stringbuffer.h"
#include "rapidjson/error/en.h"

ROSTopicHandler::ROSTopicHandler(std::shared_ptr<mqtt::async_client> mqttClient, int mqttQOS)
{
	mCli = mqttClient;
	mQOS = mqttQOS;

	ros::NodeHandle n;

	mSub_VescStatus = n.subscribe("/CAN/RX/vesc_status", 100, &ROSTopicHandler::callback_VescStatus, this);
	mTimer_VescStatus = n.createTimer(ros::Duration(mInterval_VescStatus), &ROSTopicHandler::fire_VescStatus, this);

	mMsg_ZedImuData = std::make_shared<sensor_msgs::Imu>();
	mSub_ZedImuData = n.subscribe("/zed2/zed_node/imu/data", 100, &ROSTopicHandler::callback_ZedImuData, this);
	mTimer_ZedImuData = n.createTimer(ros::Duration(mInterval_ZedImuData), &ROSTopicHandler::fire_ZedImuData, this);

	mPub_Wheels = n.advertise<can_wrapper::Wheels>("/CAN/TX/set_motor_vel", 1000);
	mPub_RoverControl = n.advertise<can_wrapper::RoverControl>("/MQTT/RoverControl", 1000);
	mPub_ManipulatorControl = n.advertise<mqtt_bridge::ManipulatorMessage>("/manipulator_joints", 1000);
	mPub_RoverStatus = n.advertise<can_wrapper::RoverStatus>("/MQTT/RoverStatus", 1000);
}

void ROSTopicHandler::publishMqttMessage(const std::string topicName, const char *message)
{
	ROS_DEBUG("Publishing MQTT message on topic [%s]: [%s]", topicName.c_str(), message);
	mqtt::message_ptr pubmsg = mqtt::make_message(topicName, message);
	pubmsg->set_qos(mQOS);
	mCli->publish(pubmsg);
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

	for (auto msgPair : mMsgMap_VescStatus)
	{
		publishMqttMessage_VescStatus(msgPair.second);
	}
	
	mMsgMap_VescStatus.clear();
}

void ROSTopicHandler::callback_VescStatus(const can_wrapper::VescStatus::ConstPtr &receivedMsg)
{
	//ROS_DEBUG("I received (ROS): a message (VescStatus)");

	std::map<int, std::shared_ptr<can_wrapper::VescStatus>>::iterator it = mMsgMap_VescStatus.find(receivedMsg->VescId);
	if (it != mMsgMap_VescStatus.end())
	{
		// if message for received VescId exists, then average the received msg with that message
		std::shared_ptr<can_wrapper::VescStatus> msg = it->second;
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
		msg->PrecisePos = (msg->PrecisePos + receivedMsg->PrecisePos) / 2;
		return;
	}

	// if message for received VescId does not exist, then make a new message and insert it into the map
	std::shared_ptr<can_wrapper::VescStatus> msg = std::make_shared<can_wrapper::VescStatus>();

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
	msg->PrecisePos = receivedMsg->PrecisePos;

	mMsgMap_VescStatus.insert({receivedMsg->VescId, msg});
}

void ROSTopicHandler::publishMqttMessage_VescStatus(std::shared_ptr<can_wrapper::VescStatus> msg)
{
	rapidjson::Document d;
	d.SetObject();

	std::map<std::string, int> jsonIntFieldsMap{{"VescId", msg->VescId}, {"ERPM", msg->ERPM}};

	std::map<std::string, double> jsonDoubleFieldsMap{
		{"Current", msg->Current}, {"DutyCycle", msg->DutyCycle}, {"AhUsed", msg->AhUsed}, {"AhCharged", msg->AhCharged}, {"WhUsed", msg->WhUsed}, {"WhCharged", msg->WhCharged}, {"TempFet", msg->TempFet}, {"TempMotor", msg->TempMotor}, {"CurrentIn", msg->CurrentIn}, {"PidPos", msg->PidPos}, {"Tachometer", msg->Tachometer}, {"VoltsIn", msg->VoltsIn}, {"ADC1", msg->ADC1}, {"ADC2", msg->ADC2}, {"ADC3", msg->ADC3}, {"PPM", msg->PPM}, {"PrecisePos", msg->PrecisePos}};

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
	if (!mFirst_ZedImuData)
	{
		publishMqttMessage_ZedImuData(mMsg_ZedImuData);
		mFirst_ZedImuData = true;
	}
}

void ROSTopicHandler::callback_ZedImuData(const sensor_msgs::Imu::ConstPtr &receivedMsg)
{
	//ROS_DEBUG("I received (ROS): a message (Imu)");

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

void ROSTopicHandler::publishMqttMessage_ZedImuData(std::shared_ptr<sensor_msgs::Imu> msg)
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

void ROSTopicHandler::publishMessage_Wheels(can_wrapper::Wheels message)
{
	mPub_Wheels.publish(message);
	ROS_DEBUG("I published (ROS): a message (Wheels)");
}

// ###### RoverControl ######

void ROSTopicHandler::publishMessage_RoverControl(can_wrapper::RoverControl message)
{
	mPub_RoverControl.publish(message);
	ROS_DEBUG("I published (ROS): a message (RoverControl)");
}

// ##### ManipulatorControl ######

void ROSTopicHandler::publishMessage_ManipulatorControl(mqtt_bridge::ManipulatorMessage message)
{
	mPub_ManipulatorControl.publish(message);
	ROS_DEBUG("I published (ROS): a message (ManipulatorControl)");
}

// ##### RoverStatus ######

void ROSTopicHandler::publishMessage_RoverStatus(can_wrapper::RoverStatus message)
{
	mPub_RoverStatus.publish(message);
	ROS_DEBUG("I published (ROS): a message (RoverStatus)");
}