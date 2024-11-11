#ifndef ROS_TOPIC_HANDLER_H
#define ROS_TOPIC_HANDLER_H

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include "can_bridge/msg/wheels.hpp"
#include "can_bridge/msg/vesc_status.hpp"
#include "can_bridge/msg/rover_control.hpp"
#include "can_bridge/msg/rover_status.hpp"
#include "mqtt_bridge/msg/manipulator_message.hpp"
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

  void publishMessage_Wheels(can_bridge::msg::Wheels message);
  void publishMessage_RoverControl(can_bridge::msg::RoverControl message);
  void publishMessage_ManipulatorControl(mqtt_bridge::msg::ManipulatorMessage message);
  void publishMessage_RoverStatus(can_bridge::msg::RoverStatus message);

private:
  void publishMqttMessage(const std::string topicName, const char *message);
  void addTimestampToJSON(rapidjson::Document &doc, long int msec);
  template<typename T>
  void addMemberToJSON(rapidjson::Document &doc, std::string name, T value);
  template<typename T>
  void addMembersFromMapToJSON(rapidjson::Document &doc, const std::map<std::string, T>& m);

  void callback_VescStatus(const can_bridge::msg::VescStatus::ConstPtr &receivedMsg);
  void fire_VescStatus();
  void publishMqttMessage_VescStatus(std::shared_ptr<can_bridge::msg::VescStatus> msg);

  void callback_ZedImuData(const sensor_msgs::msg::Imu::ConstPtr &receivedMsg);
  void fire_ZedImuData();
  void publishMqttMessage_ZedImuData(std::shared_ptr<sensor_msgs::msg::Imu> msg);


  std::shared_ptr<mqtt::async_client> mCli;
  int mQOS;

  const int32_t mInterval_VescStatus = 50;
  rclcpp::Subscription<can_bridge::msg::VescStatus>::SharedPtr mSub_VescStatus;
  rclcpp::TimerBase::SharedPtr mTimer_VescStatus;
  std::map<int, std::shared_ptr<can_bridge::msg::VescStatus>> mMsgMap_VescStatus;

  const int32_t mInterval_ZedImuData = 50;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr mSub_ZedImuData;
  rclcpp::TimerBase::SharedPtr mTimer_ZedImuData;
  std::shared_ptr<sensor_msgs::msg::Imu> mMsg_ZedImuData;
  bool mFirst_ZedImuData = true;

  rclcpp::Publisher<can_bridge::msg::Wheels>::SharedPtr mPub_Wheels;
  rclcpp::Publisher<can_bridge::msg::RoverControl>::SharedPtr mPub_RoverControl;
  rclcpp::Publisher<mqtt_bridge::msg::ManipulatorMessage>::SharedPtr mPub_ManipulatorControl;
  rclcpp::Publisher<can_bridge::msg::RoverStatus>::SharedPtr mPub_RoverStatus;

  rclcpp::CallbackGroup::SharedPtr timer_cb_group;

  rclcpp::Node::SharedPtr n;
};

#endif // ROS_TOPIC_HANDLER_H