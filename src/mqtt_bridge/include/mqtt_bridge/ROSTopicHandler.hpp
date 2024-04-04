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
  void publishMessage(mqtt_bridge::Wheels message);

private:
  void ROSTopicCallback(const mqtt_bridge::VescStatus::ConstPtr &receivedMsg);
  void ROSTopicCallback2(const sensor_msgs::Imu::ConstPtr &receivedMsg);
  void publishMqttMessage(const std::string topicName, const char *message);
  void addTimestampToJSON(rapidjson::Document &doc, long int nsec);
  ros::Publisher pub;
  ros::Subscriber sub;
  ros::Subscriber sub2;
  mqtt::async_client *cli;
  int QOS;
};

#endif // ROS_TOPIC_HANDLER_H