#ifndef CAN2ROS_H
#define CAN2ROS_H

#include <ros/ros.h>
#include <string>
#include <can_msgs/Frame.h>
#include <geometry_msgs/Point32.h>
#include <memory>
#include "can_wrapper/CanMessage.hpp"

class Can2Ros
{
private:
    ros::NodeHandle nh;
    float mRPM_scale;
    static std::unique_ptr<Can2Ros> instance;
    void handleFrame(CanMessage cm);
    geometry_msgs::Point32 decodeMotorVel(CanMessage cm);

    ros::Subscriber mRawCanSub;
    ros::Publisher mDriversLeft;
    ros::Publisher mDriversRight;
    ros::Publisher mArm123;
    ros::Publisher mArm456;

public:
    Can2Ros() = default;
    Can2Ros(Can2Ros &) = delete;
    void operator=(const Can2Ros &) = delete;

    static Can2Ros *getSingleton();
    void init(std::string can_RX_topic = "/CAN/RX/", float rpm_scale = 1);

    static void handleRosCallback(const can_msgs::Frame::ConstPtr &msg);
};

#endif // CAN2ROS_H