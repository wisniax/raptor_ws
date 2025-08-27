#ifndef ROS_TOPIC_HANDLER_H
#define ROS_TOPIC_HANDLER_H

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include "rex_interfaces/msg/wheels.hpp"
#include "rex_interfaces/msg/vesc_status.hpp"
#include "rex_interfaces/msg/rover_control.hpp"
#include "rex_interfaces/msg/rover_status.hpp"
#include "rex_interfaces/msg/sampler_control.hpp"
#include "rex_interfaces/msg/manipulator_mqtt_message.hpp"
#include "rex_interfaces/msg/battery_info.hpp"
#include "rex_interfaces/msg/sampler_feedback.hpp"
#define RAPIDJSON_HAS_STDSTRING 1
#include "rapidjson/document.h"

// forward declarations
namespace mqtt
{
  class async_client;
}

class ROSTopicHandler
{
public:
  ROSTopicHandler(std::shared_ptr<mqtt::async_client> mqttClient, int mqttQOS, rclcpp::Node::SharedPtr node);

  void publishMessage_Wheels(rex_interfaces::msg::Wheels message);
  void publishMessage_RoverControl(rex_interfaces::msg::RoverControl message);
  void publishMessage_ManipulatorControl(rex_interfaces::msg::ManipulatorMqttMessage message);
  void publishMessage_SamplerControl(rex_interfaces::msg::SamplerControl message);
  void publishMessage_RoverStatus(rex_interfaces::msg::RoverStatus message);

private:
  void publishMqttMessage(const std::string topicName, const char *message);
  void addTimestampToJSON(rapidjson::Document &doc, long int msec);
  template<typename T>
  void addMemberToJSON(rapidjson::Document &doc, std::string name, T value);
  template<typename T>
  void addMembersFromMapToJSON(rapidjson::Document &doc, const std::map<std::string, T>& m);

  void callback_VescStatus(const rex_interfaces::msg::VescStatus::ConstSharedPtr &receivedMsg);
  void fire_VescStatus();
  void publishMqttMessage_VescStatus(std::shared_ptr<rex_interfaces::msg::VescStatus> msg);

  void callback_ZedImuData(const sensor_msgs::msg::Imu::ConstSharedPtr &receivedMsg);
  void fire_ZedImuData();
  void publishMqttMessage_ZedImuData(std::shared_ptr<sensor_msgs::msg::Imu> msg);

  void callback_BatteryInfo(const rex_interfaces::msg::BatteryInfo::ConstSharedPtr &receivedMsg);
  void callback_SamplerFeedback(const rex_interfaces::msg::SamplerFeedback::ConstSharedPtr &receivedMsg);

  std::shared_ptr<mqtt::async_client> mCli;
  int mQOS;

  const int32_t mInterval_VescStatus = 50;
  rclcpp::Subscription<rex_interfaces::msg::VescStatus>::SharedPtr mSub_VescStatus;
  rclcpp::TimerBase::SharedPtr mTimer_VescStatus;
  std::map<int, std::shared_ptr<rex_interfaces::msg::VescStatus>> mMsgMap_VescStatus;

  const int32_t mInterval_ZedImuData = 50;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr mSub_ZedImuData;
  rclcpp::TimerBase::SharedPtr mTimer_ZedImuData;
  std::shared_ptr<sensor_msgs::msg::Imu> mMsg_ZedImuData;
  bool mFirst_ZedImuData = true;

  rclcpp::Subscription<rex_interfaces::msg::BatteryInfo>::SharedPtr mSub_BatteryInfo;
  rclcpp::Subscription<rex_interfaces::msg::SamplerFeedback>::SharedPtr mSub_SamplerFeedback;

  rclcpp::Publisher<rex_interfaces::msg::Wheels>::SharedPtr mPub_Wheels;
  rclcpp::Publisher<rex_interfaces::msg::RoverControl>::SharedPtr mPub_RoverControl;
  rclcpp::Publisher<rex_interfaces::msg::ManipulatorMqttMessage>::SharedPtr mPub_ManipulatorControl;
  rclcpp::Publisher<rex_interfaces::msg::SamplerControl>::SharedPtr mPub_SamplerControl;
  rclcpp::Publisher<rex_interfaces::msg::RoverStatus>::SharedPtr mPub_RoverStatus;

  rclcpp::CallbackGroup::SharedPtr timer_cb_group;

  rclcpp::Node::SharedPtr n;
};

#endif // ROS_TOPIC_HANDLER_H
