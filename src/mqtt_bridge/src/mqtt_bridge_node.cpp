#include <iostream>
#include <cstdlib>
#include <string>
#include <cstring>
#include <cctype>
#include <thread>
#include <chrono>
#include <sstream>
#include <mqtt/async_client.h>
#include <ros/console.h>
#define RAPIDJSON_HAS_STDSTRING 1
class JsonAssertException : public std::exception
{
public:
	char const *what()
	{
		return "JSON assert exception";
	}
};
// custom assert(x) for rapidjson library so that it doesn't abort the program on errors
#define RAPIDJSON_ASSERT(x) (static_cast<bool>(x) ? void(0) : throw JsonAssertException())
#include "rapidjson/document.h"
#include "rapidjson/writer.h"
#include "rapidjson/stringbuffer.h"
#include "rapidjson/error/en.h"
#include "mqtt_bridge/ROSTopicHandler.hpp"

// TODO:
// - rosdep (for paho)?
// - [X] separate into hpp/cpp
// - more generic way for handling different message types
// - wait for connection on start
// - [WiP] exception handling
// - send MQTT messages at a specified rate

ros::Time unixMillisecondsToROSTimestamp(unsigned long int msec)
{
	ros::Time timestamp;
	timestamp.fromSec(msec / (double)1000.0);
	return timestamp;
}

void processMqttWheelsMessage(const char *payloadMsg, ROSTopicHandler *rth)
{
	rapidjson::Document d;
	rapidjson::ParseResult ok = d.Parse(payloadMsg);

	if (!ok)
	{
		ROS_WARN_STREAM("JSON parse error: " << rapidjson::GetParseError_En(ok.Code()) << " (" << ok.Offset() << "), discarding MQTT message.");
	}
	else
	{
		try
		{
			mqtt_bridge::Wheels msg;

			msg.commandId = d["commandId"].GetUint();
			msg.frontLeft = d["frontLeft"].GetDouble();
			msg.frontRight = d["frontRight"].GetDouble();
			msg.rearLeft = d["rearLeft"].GetDouble();
			msg.rearRight = d["rearRight"].GetDouble();

			msg.header.stamp = unixMillisecondsToROSTimestamp(d["Timestamp"].GetUint64());

			rth->publishMessage(msg);
		}
		catch (JsonAssertException e)
		{
			ROS_WARN("JSON assert exception, discarding MQTT message.");
		}
	}
}

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "mqtt_bridge_node");

	// set ROS console logger level to DEBUG
	if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
	{
		ros::console::notifyLoggerLevelsChanged();
	}

	// ### MQTT configuration ###
	const std::string SERVER_ADDRESS("mqtt://broker.hivemq.com:1883");
	const std::string CLIENT_ID("mqtt_bridge_node_ros");
	const int MQTT_VERSION = MQTTVERSION_5;
	const int SESSION_EXPIRY = 604800;
	const int PUBLISHER_QOS = 0;
	const int KEEP_ALIVE = 20;
	const std::chrono::seconds RECONNECT_MIN_RETRY_INTERVAL{1};
	const std::chrono::seconds RECONNECT_MAX_RETRY_INTERVAL{16};
	// const bool NO_LOCAL = true;
	const bool CLEAN_START = false;

	auto SUBSCRIBED_TOPICS_NAMES = mqtt::string_collection::create({"RappTORS/Wheels", "RappTORS/Wheels2"});
	const std::vector<int> SUBSCRIBED_TOPICS_QOS{0, 0};

	mqtt::async_client *cli = new mqtt::async_client(SERVER_ADDRESS, CLIENT_ID,
													 mqtt::create_options(MQTT_VERSION));

	// auto lwt = mqtt::message(TOPIC, "LWT message", QOS, false);

	auto connOpts = mqtt::connect_options_builder()
						.properties({{mqtt::property::SESSION_EXPIRY_INTERVAL, SESSION_EXPIRY}})
						.clean_start(CLEAN_START)
						//.will(std::move(lwt))
						.keep_alive_interval(std::chrono::seconds(KEEP_ALIVE))
						.automatic_reconnect(RECONNECT_MIN_RETRY_INTERVAL, RECONNECT_MAX_RETRY_INTERVAL)
						.finalize();

	ROSTopicHandler *rth = new ROSTopicHandler(cli, PUBLISHER_QOS);

	// callback for connection lost to MQTT broker
	cli->set_connection_lost_handler([](const std::string &)
									 { ROS_ERROR("Connection to MQTT server lost. Trying to reconnect..."); });

	// callback for incoming MQTT messages
	cli->set_message_callback([rth](mqtt::const_message_ptr mqtt_msg) {
		auto messageTopic = mqtt_msg->get_topic();

		ROS_DEBUG("I received (MQTT): [%s] on topic: [%s]", mqtt_msg->get_payload_str().c_str(), messageTopic.c_str());

		if (messageTopic == "RappTORS/Wheels") {
			processMqttWheelsMessage(mqtt_msg->get_payload_str().c_str(), rth);
		} else if (messageTopic == "RappTORS/Wheels2") {
			ROS_DEBUG("Wheels2");
		} else {
			ROS_WARN_STREAM("Unknown MQTT topic: " << messageTopic << ", discarding MQTT message.");
		} });

	// Start the MQTT connection, subscribe to MQTT topics.
	try
	{
		ROS_INFO("Connecting to MQTT server...");
		auto tok = cli->connect(connOpts);
		tok->wait();
		ROS_INFO("Connected to MQTT server.");

		// auto subOpts = mqtt::subscribe_options(NO_LOCAL);
		cli->subscribe(SUBSCRIBED_TOPICS_NAMES, SUBSCRIBED_TOPICS_QOS);
	}
	catch (const mqtt::exception &exc)
	{
		ROS_FATAL_STREAM("Error connecting to MQTT server: " << exc.what());
		return 1;
	}

	ros::spin();

	// Disconnect from MQTT broker
	try
	{
		std::cout << "Disconnecting from the MQTT server..." << std::endl;
		cli->disconnect()->wait();
		std::cout << "Disconnected from the MQTT server." << std::endl;
	}
	catch (const mqtt::exception &exc)
	{
		std::cout << "Error disconnecting from MQTT server: " << exc.what() << std::endl;
		delete rth;
		delete cli;
		return 1;
	}

	delete rth;
	delete cli;
	return 0;
}
