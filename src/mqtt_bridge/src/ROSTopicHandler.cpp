/*
Copyright (c) 2024-2026 Bartosz Mazurkiewicz.
Licensed under the MIT License.
*/

#include "mqtt_bridge/ROSTopicHandler.hpp"
#include <mqtt/async_client.h>
#include "rapidjson/writer.h"
#include "rapidjson/stringbuffer.h"
#include "rapidjson/error/en.h"

ROSTopicHandler::ROSTopicHandler(std::shared_ptr<mqtt::async_client> mqttClient, int mqttQOS, rclcpp::Node* node, IJSONValidator* validator)
{
    mCli = mqttClient;
    mQOS = mqttQOS;
    n = node;
    jsonValidator = validator;

    auto timer_cb_group = n->get_node_base_interface()->get_default_callback_group();

    mSub_VescStatus = n->create_subscription<rex_interfaces::msg::VescStatus>
     ("/CAN/RX/vesc_status", 100, std::bind(&ROSTopicHandler::callback_VescStatus, this, std::placeholders::_1));
    mTimer_VescStatus = n->create_timer
     (std::chrono::milliseconds(mInterval_VescStatus), std::bind(&ROSTopicHandler::fire_VescStatus, this), timer_cb_group);

    mSub_BatteryInfo = n->create_subscription<rex_interfaces::msg::BatteryInfo>
     ("/CAN/RX/battery_info", 100, std::bind(&ROSTopicHandler::callback_BatteryInfo, this, std::placeholders::_1));
    mSub_SamplerFeedback = n->create_subscription<rex_interfaces::msg::SamplerFeedback>
     ("/CAN/RX/sampler_status", 100, std::bind(&ROSTopicHandler::callback_SamplerFeedback, this, std::placeholders::_1));

     mSub_RosoutLogs = n->create_subscription<rcl_interfaces::msg::Log>
     ("/rosout", 100, std::bind(&ROSTopicHandler::callback_RosoutLogs, this, std::placeholders::_1));

    mPub_RoverControl = n->create_publisher<rex_interfaces::msg::RoverControl>("/MQTT/RoverControl", 1000);
    mPub_SamplerControl = n->create_publisher<rex_interfaces::msg::SamplerControl>("/MQTT/SamplerControl", 1000);
    mPub_RoverStatus = n->create_publisher<rex_interfaces::msg::RoverStatus>("/MQTT/RoverStatus", 1000);
    mPub_RoboticArmControl = n->create_publisher<rex_interfaces::msg::RoboticArmControl>("/MQTT/RoboticArmControl", 1000);
    mPub_CalibrateAxis = n->create_publisher<rex_interfaces::msg::CalibrateAxis>("/MQTT/CalibrateAxis", 1000);
}

void ROSTopicHandler::publishMqttMessage(const std::string& topicName, const char* message)
{
    RCLCPP_DEBUG_THROTTLE(n->get_logger(),
     *n->get_clock(), 500, "Publishing MQTT message on topic [%s]: [%s]", topicName.c_str(), message);
    mqtt::message_ptr pubmsg = mqtt::make_message(topicName, message);
    pubmsg->set_qos(mQOS);
    mCli->publish(pubmsg);
}

// ###### json operations ######

void ROSTopicHandler::addTimestampToJSON(rapidjson::Document& doc, const builtin_interfaces::msg::Time& time)
{
    rapidjson::Value k("Timestamp", doc.GetAllocator());
    rapidjson::Value v;
    int64_t msec = ((int64_t)time.sec * 1000) + ((int64_t)time.nanosec / 1000000);
    v.SetUint64(msec);
    doc.AddMember(k, v, doc.GetAllocator());
}

template <typename T>
void ROSTopicHandler::addMemberToJSON(rapidjson::Document& doc, const std::string& name, const T& value)
{
    rapidjson::Value k(name, doc.GetAllocator());
    rapidjson::Value v(value);
    doc.AddMember(k, v, doc.GetAllocator());
}

template <typename T, typename Allocator>
void ROSTopicHandler::addMemberToJSON(rapidjson::Value& doc, const std::string& name, const T& value, Allocator& alc)
{
    rapidjson::Value k(name, alc);
    rapidjson::Value v(value);
    doc.AddMember(k, v, alc);
}

template <typename T>
void ROSTopicHandler::addMembersFromMapToJSON(rapidjson::Document& doc, const std::map<std::string, T>& m)
{
    for (const auto& n : m)
    {
        addMemberToJSON(doc, n.first, n.second);
    }
}

std::string ROSTopicHandler::getStringFromJSON(const rapidjson::Document& doc) {
    rapidjson::StringBuffer buffer;
    rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
    doc.Accept(writer);

    std::string output(buffer.GetString());
    return output;
}

// ###### VescStatus ######

void ROSTopicHandler::fire_VescStatus()
{
    for (auto msgPair : mMsgMap_VescStatus)
    {
        publishMqttMessage_VescStatus(msgPair.second);
    }

    mMsgMap_VescStatus.clear();
}

void ROSTopicHandler::callback_VescStatus(const rex_interfaces::msg::VescStatus::ConstSharedPtr& receivedMsg)
{
    std::map<int, rex_interfaces::msg::VescStatus::SharedPtr>::iterator it
     = mMsgMap_VescStatus.find(receivedMsg->vesc_id);

    if (it != mMsgMap_VescStatus.end())
    {
        // if message for received vesc_id exists, then average the received msg with that message
        rex_interfaces::msg::VescStatus::SharedPtr msg = it->second;
        // always take the most recent timestamp
        msg->header.stamp = receivedMsg->header.stamp;
        msg->erpm = (msg->erpm + receivedMsg->erpm) / 2;
        msg->current = (msg->current + receivedMsg->current) / 2;
        msg->duty_cycle = (msg->duty_cycle + receivedMsg->duty_cycle) / 2;
        msg->ah_used = (msg->ah_used + receivedMsg->ah_used) / 2;
        msg->ah_charged = (msg->ah_charged + receivedMsg->ah_charged) / 2;
        msg->wh_used = (msg->wh_used + receivedMsg->wh_used) / 2;
        msg->wh_charged = (msg->wh_charged + receivedMsg->wh_charged) / 2;
        msg->temp_fet = (msg->temp_fet + receivedMsg->temp_fet) / 2;
        msg->temp_motor = (msg->temp_motor + receivedMsg->temp_motor) / 2;
        msg->current_in = (msg->current_in + receivedMsg->current_in) / 2;
        msg->pid_pos = (msg->pid_pos + receivedMsg->pid_pos) / 2;
        msg->tachometer = (msg->tachometer + receivedMsg->tachometer) / 2;
        msg->volts_in = (msg->volts_in + receivedMsg->volts_in) / 2;
        msg->adc1 = (msg->adc1 + receivedMsg->adc1) / 2;
        msg->adc2 = (msg->adc2 + receivedMsg->adc2) / 2;
        msg->adc3 = (msg->adc3 + receivedMsg->adc3) / 2;
        msg->ppm = (msg->ppm + receivedMsg->ppm) / 2;
        msg->precise_pos = (msg->precise_pos + receivedMsg->precise_pos) / 2;
        return;
    }

    // if message for received vesc_id does not exist, then make a copy of the received msg and insert it into the map
    rex_interfaces::msg::VescStatus::SharedPtr msg = std::make_shared<rex_interfaces::msg::VescStatus>(*receivedMsg);
    mMsgMap_VescStatus.insert({msg->vesc_id, msg});
}

void ROSTopicHandler::publishMqttMessage_VescStatus(const rex_interfaces::msg::VescStatus::SharedPtr& msg)
{
    rapidjson::Document d;
    d.SetObject();

    std::map<std::string, int> jsonIntFieldsMap{{"VescId", msg->vesc_id}, {"ERPM", msg->erpm}};

    std::map<std::string, double> jsonDoubleFieldsMap{
        {"Current", msg->current}, {"DutyCycle", msg->duty_cycle}, {"AhUsed", msg->ah_used},
        {"AhCharged", msg->ah_charged}, {"WhUsed", msg->wh_used}, {"WhCharged", msg->wh_charged},
        {"TempFet", msg->temp_fet}, {"TempMotor", msg->temp_motor}, {"CurrentIn", msg->current_in},
        {"PidPos", msg->pid_pos}, {"Tachometer", msg->tachometer}, {"VoltsIn", msg->volts_in},
        {"ADC1", msg->adc1}, {"ADC2", msg->adc2}, {"ADC3", msg->adc3},
        {"PPM", msg->ppm}, {"PrecisePos", msg->precise_pos}};

    addMembersFromMapToJSON(d, jsonIntFieldsMap);

    addMembersFromMapToJSON(d, jsonDoubleFieldsMap);

    addTimestampToJSON(d, msg->header.stamp);

    publishMqttMessage("RappTORS/VescStatus", getStringFromJSON(d).c_str());
}

// ###### BatteryInfo ######
void ROSTopicHandler::callback_BatteryInfo(const rex_interfaces::msg::BatteryInfo::ConstSharedPtr& msg)
{
    rapidjson::Document d;
    d.SetObject();

    std::map<std::string, int> jsonIntFieldsMap{
        {"Slot", msg->slot}, {"ID", msg->id}, {"BatteryStatus", msg->battery_status}, {"HotswapStatus", msg->hotswap_status}};

    std::map<std::string, double> jsonDoubleFieldsMap{
        {"Voltage", msg->voltage}, {"Current", msg->current}, {"Temperature", msg->temperature}, {"ChargePercent", msg->charge_percent}};

    addMembersFromMapToJSON(d, jsonIntFieldsMap);

    addMembersFromMapToJSON(d, jsonDoubleFieldsMap);

    addTimestampToJSON(d, msg->header.stamp);

    if (!jsonValidator->validateJSON(d, "BatteryInfo")) return;

    publishMqttMessage("RappTORS/BatteryInfo", getStringFromJSON(d).c_str());
}

// ###### SamplerFeedback ######
void ROSTopicHandler::callback_SamplerFeedback(const rex_interfaces::msg::SamplerFeedback::ConstSharedPtr& msg)
{
    rapidjson::Document d;
    d.SetObject();

    std::map<std::string, double> jsonDoubleFieldsMap{
        {"WeightA", msg->weight_a}, {"WeightB", msg->weight_b}, {"WeightC", msg->weight_c}, {"Ph", msg->ph}, {"Distance", msg->distance}};

    addMembersFromMapToJSON(d, jsonDoubleFieldsMap);

    addTimestampToJSON(d, msg->header.stamp);

    if (!jsonValidator->validateJSON(d, "SamplerFeedback")) return;

    publishMqttMessage("RappTORS/SamplerFeedback", getStringFromJSON(d).c_str());
}

// ##### RosoutLogs #######
void ROSTopicHandler::callback_RosoutLogs(const rcl_interfaces::msg::Log::ConstSharedPtr& msg)
{
    std::string level;
    rapidjson::Document d;
    d.SetObject();

    addMemberToJSON(d, "level", msg->level);
    addMemberToJSON(d, "name", rapidjson::StringRef(msg->name.c_str()));
    addMemberToJSON(d, "msg", rapidjson::StringRef(msg->msg.c_str()));
    addMemberToJSON(d, "file", rapidjson::StringRef(msg->file.c_str()));
    addMemberToJSON(d, "function", rapidjson::StringRef(msg->function.c_str()));
    addMemberToJSON(d, "line", msg->line);

    addTimestampToJSON(d, msg->stamp);

    switch (msg->level) {
        case 10:
            level = "Debug";
            break;
        case 20:
            level = "Info";
            break;
        case 30:
            level = "Warn";
            break;
        case 40:
            level = "Error";
            break;
        case 50:
            level = "Fatal";
            break;
        default:
            level = "Unknown";
            break;
    }
    std::string topic = "RappTORS/Logs/" + level;

    publishMqttMessage(topic, getStringFromJSON(d).c_str());
}

// ###### RoverControl ######

void ROSTopicHandler::publishMessage_RoverControl(const rex_interfaces::msg::RoverControl& message)
{
    mPub_RoverControl->publish(message);
}

// ##### SamplerControl ######

void ROSTopicHandler::publishMessage_SamplerControl(const rex_interfaces::msg::SamplerControl& message)
{
    mPub_SamplerControl->publish(message);
}

// ##### RoverStatus ######

void ROSTopicHandler::publishMessage_RoverStatus(const rex_interfaces::msg::RoverStatus& message)
{
    mPub_RoverStatus->publish(message);
}

// ##### RoboticArmControl ######

void ROSTopicHandler::publishMessage_RoboticArmControl(const rex_interfaces::msg::RoboticArmControl& message)
{
    mPub_RoboticArmControl->publish(message);
}

// ##### CalibrateAxis ######

void ROSTopicHandler::publishMessage_CalibrateAxis(const rex_interfaces::msg::CalibrateAxis& message)
{
    mPub_CalibrateAxis->publish(message);
}
