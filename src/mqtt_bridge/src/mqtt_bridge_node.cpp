#include <iostream>
#include <cstdlib>
#include <string>
#include <cstring>
#include <cctype>
#include <thread>
#include <chrono>
#include <sstream>
#include <mqtt/async_client.h>
#include <rcutils/logging.h>

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

rclcpp::Time unixMillisecondsToROSTimestamp(unsigned long int msec)
{
	rclcpp::Time timestamp((int64_t)(msec * (double)1000000.0));
	return timestamp;
}

void processMqttWheelsMessage(const char *payloadMsg, std::shared_ptr<ROSTopicHandler> rth, std::shared_ptr<rclcpp::Node> node)
{
	rapidjson::Document d;
	rapidjson::ParseResult ok = d.Parse(payloadMsg);

	if (!ok)
	{
		RCLCPP_WARN_STREAM(node->get_logger(), "JSON parse error: " << rapidjson::GetParseError_En(ok.Code()) << " (" << ok.Offset() << "), discarding MQTT message.");
	}
	else
	{
		try
		{
			can_bridge::msg::Wheels msg;

			// msg.commandId = d["commandId"].GetUint();
			// msg.frontLeft = d["frontLeft"].GetDouble();
			// msg.frontRight = d["frontRight"].GetDouble();
			// msg.rearLeft = d["rearLeft"].GetDouble();
			// msg.rearRight = d["rearRight"].GetDouble();

			// msg.header.stamp = unixMillisecondsToROSTimestamp(d["Timestamp"].GetUint64());

			rth->publishMessage_Wheels(msg);
		}
		catch (JsonAssertException e)
		{
			RCLCPP_WARN(node->get_logger(), "JSON assert exception, discarding MQTT message.");
		}
	}
}

void processMqttRoverControlMessage(const char *payloadMsg, std::shared_ptr<ROSTopicHandler> rth, std::shared_ptr<rclcpp::Node> node)
{
	rapidjson::Document d;
	rapidjson::ParseResult ok = d.Parse(payloadMsg);

	if (!ok)
	{
		RCLCPP_WARN_STREAM(node->get_logger(), "JSON parse error: " << rapidjson::GetParseError_En(ok.Code()) << " (" << ok.Offset() << "), discarding MQTT message.");
	}
	else
	{
		try
		{
			can_bridge::msg::RoverControl msg;

			msg.vel = d["Vel"].GetDouble();
			msg.x_axis = d["XAxis"].GetDouble();
			msg.y_axis = d["YAxis"].GetDouble();
			msg.mode = d["Mode"].GetUint();

			msg.header.stamp = unixMillisecondsToROSTimestamp(d["Timestamp"].GetUint64());

			rth->publishMessage_RoverControl(msg);
		}
		catch (JsonAssertException e)
		{
			RCLCPP_WARN(node->get_logger(), "JSON assert exception, discarding MQTT message.");
		}
	}
}

void processMqttManipulatorControlMessage(const char *payloadMsg, std::shared_ptr<ROSTopicHandler> rth, std::shared_ptr<rclcpp::Node> node)
{
	rapidjson::Document d;
	rapidjson::ParseResult ok = d.Parse(payloadMsg);

	if (!ok)
	{
		RCLCPP_WARN_STREAM(node->get_logger(), "JSON parse error: " << rapidjson::GetParseError_En(ok.Code()) << " (" << ok.Offset() << "), discarding MQTT message.");
	}
	else
	{
		try
		{
			mqtt_bridge::msg::ManipulatorMessage msg;

			msg.axis1 = d["Axis1"].GetDouble();
			msg.axis2 = d["Axis2"].GetDouble();
			msg.axis3 = d["Axis3"].GetDouble();
			msg.axis4 = d["Axis4"].GetDouble();
			msg.axis5 = d["Axis5"].GetDouble();
			msg.axis6 = d["Axis6"].GetDouble();
			msg.gripper = d["Gripper"].GetDouble();

			msg.header.stamp = unixMillisecondsToROSTimestamp(d["Timestamp"].GetUint64());

			rth->publishMessage_ManipulatorControl(msg);
		}
		catch (JsonAssertException e)
		{
			RCLCPP_WARN(node->get_logger(), "JSON assert exception, discarding MQTT message.");
		}
	}
}

void processMqttSamplerControlMessage(const char *payloadMsg, std::shared_ptr<ROSTopicHandler> rth, std::shared_ptr<rclcpp::Node> node)
{
	rapidjson::Document d;
	rapidjson::ParseResult ok = d.Parse(payloadMsg);

	if (!ok)
	{
		RCLCPP_WARN_STREAM(node->get_logger(), "JSON parse error: " << rapidjson::GetParseError_En(ok.Code()) << " (" << ok.Offset() << "), discarding MQTT message.");
	}
	else
	{
		try
		{
			mqtt_bridge::msg::SamplerMessage msg;

			msg.drill_movement = d["DrillMovement"].GetDouble();
			msg.platform_movement = d["PlatformMovement"].GetDouble();
			msg.drill_action = d["DrillAction"].GetDouble();
			msg.container_degrees_0 = d["ContainerDegrees0"].GetDouble();
			msg.container_degrees_1 = d["ContainerDegrees1"].GetDouble();
			msg.container_degrees_2 = d["ContainerDegrees2"].GetDouble();

			msg.header.stamp = unixMillisecondsToROSTimestamp(d["Timestamp"].GetUint64());

			rth->publishMessage_SamplerControl(msg);
		}
		catch (JsonAssertException e)
		{
			RCLCPP_WARN(node->get_logger(), "JSON assert exception, discarding MQTT message.");
		}
	}
}

//TODO
void processMqttRoverStatusMessage(const char *payloadMsg, std::shared_ptr<ROSTopicHandler> rth, std::shared_ptr<rclcpp::Node> node)
{
	rapidjson::Document d;
	rapidjson::ParseResult ok = d.Parse(payloadMsg);

	if (!ok)
	{
		RCLCPP_WARN_STREAM(node->get_logger(), "JSON parse error: " << rapidjson::GetParseError_En(ok.Code()) << " (" << ok.Offset() << "), discarding MQTT message.");
	}
	else
	{
		try
		{
			can_bridge::msg::RoverStatus msg;

			msg.communication_state = d["CommunicationState"].GetInt();
			msg.pad_connected = d["PadConnected"].GetBool();
			msg.control_mode = d["ControlMode"].GetInt();

			msg.header.stamp = unixMillisecondsToROSTimestamp(d["Timestamp"].GetUint64());

			rth->publishMessage_RoverStatus(msg);
		}
		catch (JsonAssertException e)
		{
			RCLCPP_WARN(node->get_logger(), "JSON assert exception, discarding MQTT message.");
		}
	}
}

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("mqtt_bridge_node");

	// set ROS console logger level to INFO
	/*if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info))
	{
		ros::console::notifyLoggerLevelsChanged();
	}*/

	// ### MQTT configuration ###
	const std::string SERVER_ADDRESS("mqtt://192.168.1.20:1883");
	const std::string CLIENT_ID("mqtt_bridge_node_ros");
	const int MQTT_VERSION = MQTTVERSION_5;
	//const int SESSION_EXPIRY = 604800;
	const int PUBLISHER_QOS = 0;
	const int KEEP_ALIVE = 20;
	const std::chrono::seconds RECONNECT_MIN_RETRY_INTERVAL{1};
	const std::chrono::seconds RECONNECT_MAX_RETRY_INTERVAL{16};
	const bool CLEAN_START = false;

	auto SUBSCRIBED_TOPICS_NAMES = mqtt::string_collection::create({"RappTORS/Wheels", "RappTORS/RoverControl", "RappTORS/ManipulatorControl", "RappTORS/SamplerControl", "RappTORS/RoverStatus"});
	const std::vector<int> SUBSCRIBED_TOPICS_QOS{0, 0, 0, 0, 0};


	std::shared_ptr<mqtt::async_client> cli = std::make_shared<mqtt::async_client>(SERVER_ADDRESS, CLIENT_ID,
													 mqtt::create_options(MQTT_VERSION));


	auto connOpts = mqtt::connect_options_builder()
						//.properties({{mqtt::property::SESSION_EXPIRY_INTERVAL, SESSION_EXPIRY}})
						.clean_start(CLEAN_START)
						.keep_alive_interval(std::chrono::seconds(KEEP_ALIVE))
						.automatic_reconnect(RECONNECT_MIN_RETRY_INTERVAL, RECONNECT_MAX_RETRY_INTERVAL)
						.finalize();

	std::shared_ptr<ROSTopicHandler> rth = std::make_shared<ROSTopicHandler>(cli, PUBLISHER_QOS, node);

	// callback for connection lost to MQTT broker
	cli->set_connection_lost_handler([node](const std::string &)
									 { RCLCPP_ERROR(node->get_logger(), "Connection to MQTT server lost. Trying to reconnect..."); });

	// callback for incoming MQTT messages
	cli->set_message_callback([rth, node](mqtt::const_message_ptr mqtt_msg) {
		auto messageTopic = mqtt_msg->get_topic();

		RCLCPP_DEBUG(node->get_logger(), "I received (MQTT): [%s] on topic: [%s]", mqtt_msg->get_payload_str().c_str(), messageTopic.c_str());

		if (messageTopic == "RappTORS/Wheels") {
			processMqttWheelsMessage(mqtt_msg->get_payload_str().c_str(), rth, node);
		} else if (messageTopic == "RappTORS/RoverControl") {
			processMqttRoverControlMessage(mqtt_msg->get_payload_str().c_str(), rth, node);
		} else if (messageTopic == "RappTORS/ManipulatorControl") {
			processMqttManipulatorControlMessage(mqtt_msg->get_payload_str().c_str(), rth, node);
		} else if (messageTopic == "RappTORS/SamplerControl") {
			processMqttSamplerControlMessage(mqtt_msg->get_payload_str().c_str(), rth, node);
		} else if (messageTopic == "RappTORS/RoverStatus") {
			processMqttRoverStatusMessage(mqtt_msg->get_payload_str().c_str(), rth, node);
		} else {
			RCLCPP_WARN_STREAM(node->get_logger(), "Unknown MQTT topic: " << messageTopic << ", discarding MQTT message.");
		} });

	// loop for connecting to MQTT broker (repeat on failure)
	while (true) {
		// Start the MQTT connection, subscribe to MQTT topics.
		try
		{
			RCLCPP_INFO(node->get_logger(), "Trying to connect to MQTT server...");
			auto tok = cli->connect(connOpts);
			tok->wait();
			RCLCPP_INFO(node->get_logger(), "Successfully connected to MQTT server.");

			cli->subscribe(SUBSCRIBED_TOPICS_NAMES, SUBSCRIBED_TOPICS_QOS);
			break;
		}
		catch (const mqtt::exception &exc)
		{
			RCLCPP_ERROR_STREAM(node->get_logger(), "Error connecting to MQTT server: " << exc.what() << ", retrying in 5 seconds...");
			rclcpp::sleep_for(std::chrono::seconds(5));
			if (!rclcpp::ok()) {
				rclcpp::shutdown();
				return 1;
			}
		}
	}


	rclcpp::spin(node);

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
		return 1;
	}

	return 0;
}
