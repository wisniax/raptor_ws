#include <iostream>
#include <cstdlib>
#include <string>
#include <cstring>
#include <cctype>
#include <thread>
#include <chrono>
#include <sstream>
#include "mqtt/async_client.h"
#include "mqtt/topic.h"
#include "ros/ros.h"
#include "ros/console.h"
#include "mqtt_bridge/VescStatus.h"
#include "mqtt_bridge/Wheels.h"
#define RAPIDJSON_HAS_STDSTRING 1
#include "rapidjson/document.h"
#include "rapidjson/writer.h"
#include "rapidjson/stringbuffer.h"

// TODO:
// - rosdep (for paho)?
// - [X] timestamp in header/MQTT
// - separate into hpp/cpp
// - [X] Wheels messages
// - more generic way for handling different message types?
// - wait for connection on start
// - exception handling

class ROSTopicHandler {
  public:
    ROSTopicHandler(mqtt::topic* topic);
	void publishMessage(mqtt_bridge::Wheels message);
  private:
    void ROSTopicCallback(const mqtt_bridge::VescStatus::ConstPtr& receivedMsg);
	ros::Publisher pub;
    ros::Subscriber sub;
	mqtt::topic* mqttTopic;
};

ROSTopicHandler::ROSTopicHandler(mqtt::topic* topic) {
  ros::NodeHandle n;
  pub = n.advertise<mqtt_bridge::Wheels>("/CAN/TX/set_motor_vel", 1000);
  sub = n.subscribe("/CAN/RX/vesc_status", 1000, &ROSTopicHandler::ROSTopicCallback, this);
  mqttTopic = topic;
}

void ROSTopicHandler::ROSTopicCallback(const mqtt_bridge::VescStatus::ConstPtr& receivedMsg)
{
  //ROS_INFO("I received (ROS): [%s]", receivedMsg->header.c_str());
  ROS_DEBUG("I received (ROS): a message");

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

	for (int i = 0; i < jsonIntFields; i++) {
		rapidjson::Value k(jsonIntFieldNames[i], d.GetAllocator());
		rapidjson::Value v;
		v.SetInt(jsonIntFieldValues[i]);
		d.AddMember(k, v, d.GetAllocator());
	}

	for (int i = 0; i < jsonDoubleFields; i++) {
		rapidjson::Value k(jsonDoubleFieldNames[i], d.GetAllocator());
		rapidjson::Value v;
		v.SetDouble(jsonDoubleFieldValues[i]);
		d.AddMember(k, v, d.GetAllocator());
	}

	// timestamp
	{
		rapidjson::Value k("Timestamp", d.GetAllocator());
		rapidjson::Value v;
		v.SetUint64(receivedMsg->header.stamp.toNSec()/1000000);
		d.AddMember(k, v, d.GetAllocator());
	}

    rapidjson::StringBuffer buffer;
    rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
    d.Accept(writer);
  
  mqttTopic->publish(buffer.GetString());
  
  ROS_DEBUG("I published (MQTT): [%s]", buffer.GetString());
}

void ROSTopicHandler::publishMessage(mqtt_bridge::Wheels message)
{
  pub.publish(message);
  
  //ROS_INFO("I published (ROS): [%s]", message.c_str());
  ROS_DEBUG("I published (ROS): a message");
}


int main(int argc, char* argv[])
{
  	ros::init(argc, argv, "mqtt_bridge_node");

	if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
   		ros::console::notifyLoggerLevelsChanged();
	}


	const std::string SERVER_ADDRESS("mqtt://broker.hivemq.com:1883");
	const std::string CLIENT_ID("mqtt_bridge_node_ros");
	const std::string TOPIC("RappTORS/VescStatus");
	const std::string TOPIC2("RappTORS/Wheels");
	const int MQTT_VERSION = MQTTVERSION_5;
	const int SESSION_EXPIRY = 604800;
	const int QOS = 0;
	const int KEEP_ALIVE = 20;
	const std::chrono::seconds RECONNECT_MIN_RETRY_INTERVAL{1};
	const std::chrono::seconds RECONNECT_MAX_RETRY_INTERVAL{16};
	const bool NO_LOCAL = true;
	const bool CLEAN_START = false;


	mqtt::async_client cli(SERVER_ADDRESS, CLIENT_ID,
						   mqtt::create_options(MQTT_VERSION));

	//auto lwt = mqtt::message(TOPIC, "LWT message", QOS, false);

	auto connOpts = mqtt::connect_options_builder()
		.properties({
			{mqtt::property::SESSION_EXPIRY_INTERVAL, SESSION_EXPIRY}
		})
		.clean_start(CLEAN_START)
		//.will(std::move(lwt))
		.keep_alive_interval(std::chrono::seconds(KEEP_ALIVE))
		.automatic_reconnect(RECONNECT_MIN_RETRY_INTERVAL, RECONNECT_MAX_RETRY_INTERVAL)
		.finalize();


	mqtt::topic* topic = new mqtt::topic { cli, TOPIC, QOS };
	mqtt::topic* topic2 = new mqtt::topic { cli, TOPIC2, QOS };

	ROSTopicHandler* rth = new ROSTopicHandler(topic);

	// callback for connection lost
	cli.set_connection_lost_handler([](const std::string&) {
		ROS_ERROR("Connection to MQTT server lost. Trying to reconnect...");
	});

	// callback for incoming messages
	cli.set_message_callback([rth](mqtt::const_message_ptr mqtt_msg) {
		ROS_DEBUG("I received (MQTT): [%s]", mqtt_msg->get_payload_str().c_str());
		rapidjson::Document d;
    	d.Parse(mqtt_msg->get_payload_str().c_str());

		mqtt_bridge::Wheels msg;

		msg.commandId = d["commandId"].GetUint();
		msg.frontLeft = d["frontLeft"].GetDouble();
		msg.frontRight = d["frontRight"].GetDouble();
		msg.rearLeft = d["rearLeft"].GetDouble();
		msg.rearRight = d["rearRight"].GetDouble();

		msg.header.stamp.nsec = d["Timestamp"].GetUint64()*1000;

		rth->publishMessage(msg);
	});


	// Start the connection.
	try {
		ROS_INFO("Connecting to MQTT server...");
		auto tok = cli.connect(connOpts);
		tok->wait();
		ROS_INFO("Connected to MQTT server.");

		auto subOpts = mqtt::subscribe_options(NO_LOCAL);
		topic->subscribe(subOpts)->wait();
		topic2->subscribe(subOpts)->wait();
	}
	catch (const mqtt::exception& exc) {
		ROS_FATAL_STREAM("Error connecting to MQTT server: " << exc.what());
		return 1;
	}

	ros::spin();

	// Disconnect
	try {
		std::cout << "Disconnecting from the MQTT server..." << std::endl;
		cli.disconnect()->wait();
		std::cout << "Disconnected from the MQTT server." << std::endl;
	}
	catch (const mqtt::exception& exc) {
		std::cout << "Error disconnecting from MQTT server: " << exc.what() << std::endl;
		delete rth;
		return 1;
	}

	delete rth;
 	return 0;
}
