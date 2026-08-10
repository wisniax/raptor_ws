#ifndef MQTT_BRIDGE__ROSTOPICHANDLER_HPP_
#define MQTT_BRIDGE__ROSTOPICHANDLER_HPP_

/*
Copyright (c) 2024-2026 Bartosz Mazurkiewicz.
Licensed under the MIT License.
*/

#include <string>
#include <memory>
#include <map>

#include <rclcpp/rclcpp.hpp>

#include "rex_interfaces/msg/vesc_status.hpp"
#include "rex_interfaces/msg/rover_control.hpp"
#include "rex_interfaces/msg/rover_status.hpp"
#include "rex_interfaces/msg/sampler_control.hpp"
#include "rex_interfaces/msg/robotic_arm_control.hpp"
#include "rex_interfaces/msg/battery_info.hpp"
#include "rex_interfaces/msg/sampler_feedback.hpp"
#include "rex_interfaces/msg/calibrate_axis.hpp"
#include "rcl_interfaces/msg/log.hpp"

#include "RapidJsonConfig.hpp"
#include "rapidjson/document.h"

#include "IJSONValidator.hpp"

// forward declarations
namespace mqtt
{
class async_client;
}

class ROSTopicHandler
{
public:
  ROSTopicHandler(std::shared_ptr<mqtt::async_client> mqttClient, int mqttQOS, rclcpp::Node* node, IJSONValidator* validator);

  void publishMessage_RoverControl(const rex_interfaces::msg::RoverControl& message);
  void publishMessage_RoboticArmControl(const rex_interfaces::msg::RoboticArmControl& message);
  void publishMessage_SamplerControl(const rex_interfaces::msg::SamplerControl& message);
  void publishMessage_RoverStatus(const rex_interfaces::msg::RoverStatus& message);
  void publishMessage_CalibrateAxis(const rex_interfaces::msg::CalibrateAxis& message);

private:
  void publishMqttMessage(const std::string& topicName, const char* message);
  static void addTimestampToJSON(rapidjson::Document& doc, const builtin_interfaces::msg::Time& time);
  template <typename T>
  static void addMemberToJSON(rapidjson::Document& doc, const std::string& name, const T& value);
  template <typename T, typename Allocator>
  static void addMemberToJSON(rapidjson::Value& doc, const std::string& name, const T& value, Allocator& alc);
  template<typename T>
  static void addMembersFromMapToJSON(rapidjson::Document& doc, const std::map<std::string, T>& m);
  static std::string getStringFromJSON(const rapidjson::Document& doc);

  void callback_VescStatus(const rex_interfaces::msg::VescStatus::ConstSharedPtr& receivedMsg);
  void fire_VescStatus();
  void publishMqttMessage_VescStatus(const rex_interfaces::msg::VescStatus::SharedPtr& msg);

  void callback_BatteryInfo(const rex_interfaces::msg::BatteryInfo::ConstSharedPtr& receivedMsg);
  void callback_SamplerFeedback(const rex_interfaces::msg::SamplerFeedback::ConstSharedPtr& receivedMsg);

  void callback_RosoutLogs(const rcl_interfaces::msg::Log::ConstSharedPtr& receivedMsg);

  std::shared_ptr<mqtt::async_client> mCli;
  int mQOS;

  const int32_t mInterval_VescStatus = 50;
  rclcpp::Subscription<rex_interfaces::msg::VescStatus>::SharedPtr mSub_VescStatus;
  rclcpp::TimerBase::SharedPtr mTimer_VescStatus;
  std::map<int, rex_interfaces::msg::VescStatus::SharedPtr> mMsgMap_VescStatus;

  rclcpp::Subscription<rex_interfaces::msg::BatteryInfo>::SharedPtr mSub_BatteryInfo;
  rclcpp::Subscription<rex_interfaces::msg::SamplerFeedback>::SharedPtr mSub_SamplerFeedback;
  rclcpp::Subscription<rcl_interfaces::msg::Log>::SharedPtr mSub_RosoutLogs;

  rclcpp::Publisher<rex_interfaces::msg::RoverControl>::SharedPtr mPub_RoverControl;
  rclcpp::Publisher<rex_interfaces::msg::RoboticArmControl>::SharedPtr mPub_RoboticArmControl;
  rclcpp::Publisher<rex_interfaces::msg::SamplerControl>::SharedPtr mPub_SamplerControl;
  rclcpp::Publisher<rex_interfaces::msg::RoverStatus>::SharedPtr mPub_RoverStatus;
  rclcpp::Publisher<rex_interfaces::msg::CalibrateAxis>::SharedPtr mPub_CalibrateAxis;

  rclcpp::Node* n;
  IJSONValidator* jsonValidator;
};

#endif  // MQTT_BRIDGE__ROSTOPICHANDLER_HPP_
