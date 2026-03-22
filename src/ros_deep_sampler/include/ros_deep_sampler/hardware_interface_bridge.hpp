#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "rex_interfaces/msg/sampler_control.hpp"
#include "rex_interfaces/msg/sampler_feedback.hpp"
// #include "ros_deep_sampler/linear_movement_client.hpp"
// #include "ros_deep_sampler/linear_movement_server.hpp"
#include <chrono>
#include <string>
#include <iostream>



#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>


namespace ros_deep_sampler{

class HardwareBridge: public rclcpp::Node{
    public:
        explicit HardwareBridge(const rclcpp::NodeOptions & options);


        void publishSamplerCommand();

        void FeedbackCallback(const rex_interfaces::msg::SamplerFeedback msg);

        void trajCallback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg);

    private:

    rclcpp::Subscription<rex_interfaces::msg::SamplerFeedback>::SharedPtr SamplerFeedback_;
    rclcpp::Publisher<rex_interfaces::msg::SamplerControl>::SharedPtr SamplerCommand_;
    rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr  TrajectoryCom_;

    rclcpp::TimerBase::SharedPtr mResenderTimer;



};



}