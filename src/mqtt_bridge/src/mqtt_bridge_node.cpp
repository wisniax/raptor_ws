#include <iostream>
#include <cstdlib>
#include <string>
#include <fstream>
#include <cstring>
#include <cctype>
#include <thread>
#include <chrono>
#include <sstream>
#include <mqtt/async_client.h>
#include <rcutils/logging.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

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
			rex_interfaces::msg::Wheels msg;

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
			rex_interfaces::msg::RoverControl msg;

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
			rex_interfaces::msg::ManipulatorMqttMessage msg;

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
			rex_interfaces::msg::SamplerControl msg;

			msg.drill_movement = d["DrillMovement"].GetDouble();
			msg.platform_movement = d["PlatformMovement"].GetDouble();
			msg.drill_action = d["DrillAction"].GetDouble();
			msg.container_degrees_a = d["ContainerDegrees0"].GetDouble();
			msg.container_degrees_b = d["ContainerDegrees1"].GetDouble();
			msg.vacuum_suction = d["VacuumSuction"].GetDouble();
			msg.vacuum_a = d["VacuumA"].GetDouble();
			msg.vacuum_b = d["VacuumB"].GetDouble();
			// msg.container_degrees_2 = d["ContainerDegrees2"].GetDouble();

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
			rex_interfaces::msg::RoverStatus msg;

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

	// ### MQTT configuration ###
	const std::string SERVER_ADDRESS("ssl://localhost:8883");
	const std::string CLIENT_ID("mqtt_bridge_node_ros");
	const std::string MQTT_USERNAME("raptors");
	const std::string MQTT_PASSWORD("changeme");
	const bool ENABLE_SERVER_CERT_AUTH = false;
	// if ENABLE_SERVER_CERT_AUTH is true, but file at SSL_CA_PATH cannot be opened, system certificates are used
	std::string SSL_CA_PATH = "/opt/share/raptor_ws_mqtt_certs/ca.crt";
	const int MQTT_VERSION = MQTTVERSION_5;
	//const int SESSION_EXPIRY = 604800;
	const int PUBLISHER_QOS = 0;
	const int KEEP_ALIVE = 20;
	const std::chrono::seconds RECONNECT_MIN_RETRY_INTERVAL{1};
	const std::chrono::seconds RECONNECT_MAX_RETRY_INTERVAL{16};
	const bool CLEAN_START = false;


	auto SUBSCRIBED_TOPICS_NAMES = mqtt::string_collection::create({"RappTORS/Wheels", "RappTORS/RoverControl", "RappTORS/ManipulatorControl", "RappTORS/SamplerControl", "RappTORS/RoverStatus"});
	const std::vector<int> SUBSCRIBED_TOPICS_QOS{0, 0, 0, 0, 0};

	auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    param_desc.read_only = true;

	node->declare_parameter("mqtt_server_address", SERVER_ADDRESS, param_desc);
	node->declare_parameter("mqtt_client_id", CLIENT_ID, param_desc);
	node->declare_parameter("mqtt_enable_server_cert_auth", ENABLE_SERVER_CERT_AUTH, param_desc);

	bool tstore_found = false;
	if (node->get_parameter("mqtt_enable_server_cert_auth").as_bool()) {
		std::ifstream tstore(SSL_CA_PATH);
		if (!tstore) {
			RCLCPP_WARN_STREAM(node->get_logger(), "The trust store cannot be opened: " << SSL_CA_PATH << ", not passing it as connection parameter!");
		} else {
			RCLCPP_INFO_STREAM(node->get_logger(), "Trust store found at: " << SSL_CA_PATH);
			tstore_found = true;
		}
	} else {
		RCLCPP_WARN_STREAM(node->get_logger(), "Verification of server certificate is disabled, consider adjusting your setup!");
	}

	auto sslopts_builder = mqtt::ssl_options_builder()
						.enable_server_cert_auth(node->get_parameter("mqtt_enable_server_cert_auth").as_bool())
						.error_handler([/*node*/](const std::string& msg) {
							//RCLCPP_ERROR_STREAM(node->get_logger(), "SSL Error: " << msg);
							std::cout << "SSL Error: " << msg << ", stopping mqtt-bridge!" << std::endl;
							return 1;
						});
	if (tstore_found) sslopts_builder = sslopts_builder.trust_store(SSL_CA_PATH);
	auto sslopts = sslopts_builder.finalize();

	std::shared_ptr<mqtt::async_client> cli = std::make_shared<mqtt::async_client>(node->get_parameter("mqtt_server_address").as_string(), node->get_parameter("mqtt_client_id").as_string(),
													 mqtt::create_options(MQTT_VERSION));

	RCLCPP_DEBUG_STREAM(node->get_logger(), "Taking rosparam into account, assuming MQTT broker address: " << node->get_parameter("mqtt_server_address").as_string() << ", client id: " << node->get_parameter("mqtt_client_id").as_string());

	auto connOpts = mqtt::connect_options_builder()
						//.properties({{mqtt::property::SESSION_EXPIRY_INTERVAL, SESSION_EXPIRY}})
						.user_name(MQTT_USERNAME)
						.password(MQTT_PASSWORD)
						.ssl(std::move(sslopts))
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
