#ifndef ROS_TOPIC_HANDLER_H
#define ROS_TOPIC_HANDLER_H

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <sensor_msgs/Imu.h>
#include "mqtt_bridge/Wheels.h"
#include "mqtt_bridge/VescStatus.h"
#define RAPIDJSON_HAS_STDSTRING 1
#include "rapidjson/document.h"
#include <boost/circular_buffer.hpp>

// forward declarations
namespace mqtt
{
  class async_client;
}

class ROSTopicHandler
{
public:
  ROSTopicHandler(std::shared_ptr<mqtt::async_client> mqttClient, int mqttQOS);
  ~ROSTopicHandler();

  void publishMessage_Wheels(mqtt_bridge::Wheels message);

private:
  typedef std::shared_ptr<boost::circular_buffer<mqtt_bridge::VescStatus::ConstPtr>> VescStatusBufferPtr;

  void publishMqttMessage(const std::string topicName, const char *message);
  void addTimestampToJSON(rapidjson::Document &doc, long int nsec);
  template<typename T>
  void addMemberToJSON(rapidjson::Document &doc, std::string name, T value);
  template<typename T>
  void addMembersFromMapToJSON(rapidjson::Document &doc, const std::map<std::string, T>& m);

  std::shared_ptr<mqtt::async_client> cli;
  int QOS;

  void ROSTopicCallback_VescStatus(const mqtt_bridge::VescStatus::ConstPtr &receivedMsg);
  void ROSTopicCallback_ZedImuData(const sensor_msgs::Imu::ConstPtr &receivedMsg);

  ros::Subscriber sub_VescStatus;
  ros::Timer timer_VescStatus;
  std::map<int, VescStatusBufferPtr> bufferMap_VescStatus;
  void fire_VescStatus(const ros::TimerEvent& event);
  void PublishMqttMessage_VescStatus(mqtt_bridge::VescStatus &msg);

  ros::Subscriber sub_ZedImuData;

  ros::Publisher pub_Wheels;
};

#endif // ROS_TOPIC_HANDLER_H