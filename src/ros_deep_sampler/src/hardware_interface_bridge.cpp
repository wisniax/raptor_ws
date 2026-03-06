#include "ros_deep_sampler/hardware_interface_bridge.hpp"



namespace ros_deep_sampler{

    HardwareBridge::HardwareBridge(const rclcpp::NodeOptions & options)
  : Node("Mission_control_node", options)
  {
    SamplerCommand_ = this->create_publisher<rex_interfaces::msg::SamplerControl>(
        "/MQTT/SamplerControl", 100);

    TrajectoryCom_ = this->create_subscription<trajectory_msgs::msg::JointTrajectory>(
        "platform_controller/joint_trajectory", 10,
        std::bind(&HardwareBridge::trajCallback, this, std::placeholders::_1));

    SamplerFeedback_ = this->create_subscription<rex_interfaces::msg::SamplerFeedback>(
        "CAN/RX/sampler_status", 10,
        std::bind(&HardwareBridge::FeedbackCallback, this, std::placeholders::_1));

    mResenderTimer = this->create_timer(std::chrono::milliseconds(50), 
        std::bind(&HardwareBridge::publishSamplerCommand, this));

  }


  void HardwareBridge::publishSamplerCommand(){
    
    }

  void  HardwareBridge::FeedbackCallback(const rex_interfaces::msg::SamplerFeedback msg){

  }

  void HardwareBridge::trajCallback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg){

  }





}