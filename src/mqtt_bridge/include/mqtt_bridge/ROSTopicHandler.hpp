#ifndef ROS_TOPIC_HANDLER_H
#define ROS_TOPIC_HANDLER_H

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include "can_wrapper/Wheels.h"
#include "can_wrapper/VescStatus.h"
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

  void publishMessage_Wheels(can_wrapper::Wheels message);

private:
  void publishMqttMessage(const std::string topicName, const char *message);
  void addTimestampToJSON(rapidjson::Document &doc, long int nsec);
  template<typename T>
  void addMemberToJSON(rapidjson::Document &doc, std::string name, T value);
  template<typename T>
  void addMembersFromMapToJSON(rapidjson::Document &doc, const std::map<std::string, T>& m);

  std::shared_ptr<mqtt::async_client> mCli;
  int mQOS;

  const double mInterval_VescStatus = 0.05;
  ros::Subscriber mSub_VescStatus;
  ros::Timer mTimer_VescStatus;
  std::map<int, std::shared_ptr<can_wrapper::VescStatus>> mMsgMap_VescStatus;
  void callback_VescStatus(const can_wrapper::VescStatus::ConstPtr &receivedMsg);
  void fire_VescStatus(const ros::TimerEvent& event);
  void publishMqttMessage_VescStatus(std::shared_ptr<can_wrapper::VescStatus> msg);

  const double mInterval_ZedImuData = 0.05;
  ros::Subscriber mSub_ZedImuData;
  ros::Timer mTimer_ZedImuData;
  std::shared_ptr<sensor_msgs::Imu> mMsg_ZedImuData;
  bool mFirst_ZedImuData = true;
  void callback_ZedImuData(const sensor_msgs::Imu::ConstPtr &receivedMsg);
  void fire_ZedImuData(const ros::TimerEvent& event);
  void publishMqttMessage_ZedImuData(std::shared_ptr<sensor_msgs::Imu> msg);

  ros::Publisher mPub_Wheels;
};

#endif // ROS_TOPIC_HANDLER_H