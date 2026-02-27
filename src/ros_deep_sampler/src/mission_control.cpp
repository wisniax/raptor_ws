#include "ros_deep_sampler/mission_control.hpp"



namespace ros_deep_sampler{

    MissionControl::MissionControl(const rclcpp::NodeOptions & options)
  : Node("Mission_control_node", options)
  {

    missionTimer_ = this->create_timer(std::chrono::milliseconds(10), std::bind(&MissionControl::statesLoop, this));
    move_client_ = std::make_shared<MoveLinearActionClient>(
    rclcpp::NodeOptions());
    
    
    sub_ = this->create_subscription<std_msgs::msg::String>(
    "/command",
    10,
    std::bind(&MissionControl::MissionCheck, this, std::placeholders::_1));   
    // SubStatus = this->create_subscription<rex_interfaces::msg::SamplerControl>(
    //     "/MQTT/SamplerControl",
    //     rclcpp::QoS(10), std::bind(&MissionControl::MissionCheck, this, std::placeholders::_1));

    PubFeedback = this->create_publisher<rex_interfaces::msg::SamplerFeedback>("/MQTT/SamplerFeedback", 100);
    AppFeedbackTimer_ = this->create_wall_timer(
    std::chrono::milliseconds(500), std::bind(&MissionControl::AppFeedbackPublish, this));

    // mPubFeedback.platform_position = this->platform_position;
    // mPubFeedback.drill_position = this->drill_position;
    // mPubFeedback.drill_current = this->drill_velocity; 
    
    
  }


void MissionControl::statesLoop(){
    switch(state_){
        case State::IDLE:
        // waiting for mission_start callback
        break;

      case State::MOVING:

        break;

      case State::DRILLING:
        break;

      case State::DONE:
        break;
    }
}


void MissionControl::MissionCheck(std_msgs::msg::String::SharedPtr msg){
  RCLCPP_INFO(this->get_logger(), "Received: %s", msg->data.c_str());

    if (msg->data == "mission_start"){
     
        if(state_ == State::IDLE){
            state_ = State::MOVING;
            RCLCPP_INFO(this->get_logger(), "Current state: %s", to_string(state_).c_str());            
            move_client_->send_goal();
        }

    }
  
  
  //RCLCPP_INFO(this->get_logger(), "Current state: %s", to_string(state_).c_str());
}


void MissionControl::AppFeedbackPublish(){

  if(state_ != State::IDLE){
  // RCLCPP_INFO(this->get_logger(), "<====Publishing sampler state====> \n");
  // RCLCPP_INFO(this->get_logger(), "Platform position: '%s' \n",  std::to_string(mPubFeedback.platform_position).c_str());
  // RCLCPP_INFO(this->get_logger(), "Drill position: '%s' \n", std::to_string(mPubFeedback.drill_position).c_str());
  // RCLCPP_INFO(this->get_logger(), "Drill velocity: '%s' \n", std::to_string(mPubFeedback.drill_current).c_str());
  // PubFeedback->publish(mPubFeedback);
  }
}




} //name space 




