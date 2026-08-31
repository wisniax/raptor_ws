#ifndef MQTT_BRIDGE__MQTTBRIDGE_HPP_
#define MQTT_BRIDGE__MQTTBRIDGE_HPP_

/*
Copyright (c) 2024-2026 Bartosz Mazurkiewicz.
Licensed under the MIT License.
*/

#include <iostream>
#include <cstdlib>
#include <string>
#include <fstream>
#include <cstring>
#include <cctype>
#include <thread>
#include <chrono>
#include <sstream>
#include <cstdint>
#include <vector>
#include <memory>
#include <map>
#include <tuple>

#include <mqtt/async_client.h>
#include <mqtt/iaction_listener.h>

#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rcutils/logging.h>

#include "RapidJsonConfig.hpp"
#include "rapidjson/document.h"
#include "rapidjson/writer.h"
#include "rapidjson/stringbuffer.h"
#include "rapidjson/error/en.h"
#include "rapidjson/schema.h"
#include <rapidjson/istreamwrapper.h>

#include "IJSONValidator.hpp"
#include "ROSTopicHandler.hpp"

class MqttBridge : public rclcpp::Node, public mqtt::iaction_listener, public IJSONValidator {
    private:
    // ### MQTT configuration ###
    static inline const std::string SERVER_ADDRESS = "ssl://localhost:8883";
    static inline const std::string CLIENT_ID = "mqtt_bridge_node_ros";
    static inline const std::string MQTT_USERNAME = "raptors";
    static inline const std::string MQTT_PASSWORD = "changeme";
    const bool ENABLE_SERVER_CERT_AUTH = false;
    // if ENABLE_SERVER_CERT_AUTH is true,
    // but file at SSL_CA_PATH cannot be opened, system certificates are used
    static inline const std::string SSL_CA_PATH = "/opt/share/raptor_ws_mqtt_certs/ca.crt";
    const int MQTT_VERSION = MQTTVERSION_5;
    const int PUBLISHER_QOS = 0;
    const int KEEP_ALIVE = 20;
    const std::chrono::seconds RECONNECT_MIN_RETRY_INTERVAL{1};
    const std::chrono::seconds RECONNECT_MAX_RETRY_INTERVAL{16};
    const bool CLEAN_START = false;
    inline static const mqtt::string_collection::ptr_t SUBSCRIBED_TOPICS_NAMES =
        mqtt::string_collection::create({
            "RappTORS/RoverControl",
            "RappTORS/RoboticArmControl",
            "RappTORS/SamplerControl",
            "RappTORS/RoverStatus",
            "RappTORS/CalibrateAxis",
            "RappTORS/RoboticArmAutonomy"
        });
    const std::vector<int> SUBSCRIBED_TOPICS_QOS{0, 0, 0, 0, 0, 0};

    public:
    explicit MqttBridge(const rclcpp::NodeOptions& options);
    ~MqttBridge();
    void on_success(const mqtt::token& asyncActionToken);
    void on_failure(const mqtt::token& asyncActionToken);
    bool validateJSON(rapidjson::Document& doc, const std::string& schemaKey);

    private:
    std::shared_ptr<ROSTopicHandler> rth;
    std::shared_ptr<mqtt::async_client> cli;
    rclcpp::TimerBase::SharedPtr mTimer_Connect;
    mqtt::connect_options connOpts;

    const std::map<std::string, std::string> jsonSchemaFiles = {
        {"BatteryInfo", "batteryinfo.json"},
        {"ServoStatus", "servostatus.json"},
        {"RoverControl", "rovercontrol.json"},
        {"RoverStatus", "roverstatus.json"},
        {"SamplerControl", "samplercontrol.json"},
        {"SamplerFeedback", "samplerfeedback.json"},
        {"VescStatus", "vescstatus.json"},
        {"CalibrateAxis", "calibrateaxis.json"},
        {"RoboticArmAutonomy", "roboticarmautonomy.json"}
    };
    std::map<std::string,
     std::tuple<std::shared_ptr<rapidjson::SchemaDocument>, std::shared_ptr<rapidjson::SchemaValidator>,
      std::shared_ptr<std::atomic<bool>>>> jsonSchemaValidators;

    void setupJsonSchemaValidators();

    void tryConnect();
    void configure_client();
    bool disconnect();

    static rclcpp::Time unixMillisecondsToROSTimestamp(u_int64_t msec);
    int sslErrorCallback(const std::string& msg);
    void mqttMessageCallback(const mqtt::const_message_ptr& mqtt_msg);
    void processMqttMessage(const mqtt::string& messageTopic, const char* payloadMsg);

    bool validateRoverStatusControlMode(int32_t control_mode);
    rex_interfaces::msg::RoverControl createRoverControlMsg(const rapidjson::Document& d);
    rex_interfaces::msg::RoboticArmControl createRoboticArmControlMsg(const rapidjson::Document& d);
    rex_interfaces::msg::SamplerControl createSamplerControlMsg(const rapidjson::Document& d);
    rex_interfaces::msg::RoverStatus createRoverStatusMsg(const rapidjson::Document& d);
    rex_interfaces::msg::CalibrateAxis createCalibrateAxisMsg(const rapidjson::Document& d);
    rex_interfaces::msg::RoboticArmAutonomy createRoboticArmAutonomyMsg(const rapidjson::Document& d);
};

#endif  // MQTT_BRIDGE__MQTTBRIDGE_HPP_
