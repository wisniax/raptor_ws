#ifndef ROS_TOPIC_HANDLER_H
#define ROS_TOPIC_HANDLER_H

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include "mqtt_bridge/Wheels.h"
#include "mqtt_bridge/VescStatus.h"
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
  ROSTopicHandler(mqtt::async_client *cli, int QOS);
  void publishMessage_Wheels(mqtt_bridge::Wheels message);

private:
  void ROSTopicCallback_VescStatus(const mqtt_bridge::VescStatus::ConstPtr &receivedMsg);
  void ROSTopicCallback_ZedImuData(const sensor_msgs::Imu::ConstPtr &receivedMsg);
  void publishMqttMessage(const std::string topicName, const char *message);
  void addTimestampToJSON(rapidjson::Document &doc, long int nsec);
  template<typename T>
  void addMemberToJSON(rapidjson::Document &doc, std::string name, T value);
  template<typename T>
  void addMembersFromMapToJSON(rapidjson::Document &doc, const std::map<std::string, T>& m);
  ros::Publisher pub_Wheels;
  ros::Subscriber sub_VescStatus;
  ros::Subscriber sub_ZedImuData;
  mqtt::async_client *cli;
  int QOS;
};

#endif // ROS_TOPIC_HANDLER_H