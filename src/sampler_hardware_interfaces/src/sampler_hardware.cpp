// Copyright 2025 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/* Author: Jafar Abdi */
#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>
#include <set>
#include <string>
#include <vector>

#include <angles/angles.h>
#include <sampler_hardware_interfaces/sampler_hardware.hpp>
#include <rclcpp/executors.hpp>
#include <libVescCan/VESC_Consts.h>



namespace
{
/** @brief Sums the total rotation for joint states that wrap from 2*pi to -2*pi
when rotating in the positive direction */
double sumRotationFromMinus2PiTo2Pi(const double current_wrapped_rad, double total_rotation_in)
{
  double delta = 0;
  angles::shortest_angular_distance_with_large_limits(total_rotation_in, current_wrapped_rad, 2 * M_PI, -2 * M_PI,
                                                      delta);

  // Add the corrected delta to the total rotation
  return total_rotation_in + delta;
}
}  // namespace

namespace sampler_hardware_interfaces
{

static constexpr std::size_t POSITION_INTERFACE_INDEX = 0;
static constexpr std::size_t VELOCITY_INTERFACE_INDEX = 1;
// JointState doesn't contain an acceleration field, so right now it's not used
static constexpr std::size_t EFFORT_INTERFACE_INDEX = 3;

CallbackReturn SamplerHardware::on_init(const hardware_interface::HardwareComponentInterfaceParams& params)
{
  if (hardware_interface::SystemInterface::on_init(params) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  joint_to_can_id_["platform_joint"] = 0x80;
  joint_to_can_id_["drill_joint"]    = 0x81;
  joint_to_can_id_["rotor_joint"]    = 0x82;
  //cmd_ = std::make_shared<SamplerCanCmd>();
  feedback_ = std::make_shared<SamplerCanFeedback>();
  LastStatusMsg = std::make_shared<const RoverStatusMsg>();

  return CallbackReturn::SUCCESS;
}

void SamplerHardware::publish_platform_cmd(){


}

CallbackReturn SamplerHardware::on_configure(
    const rclcpp_lifecycle::State &)
{
    if (get_node())
    {
        platform_cmd_pub_ =
            get_node()->create_publisher<std_msgs::msg::Float64>(
                "/platform_joint_cmd", 10);
        
        drill_cmd_pub_ =
            get_node()->create_publisher<std_msgs::msg::Float64>(
                "/drill_joint_cmd", 10);

        container_cmd_pub_ = 
            get_node()->create_publisher<std_msgs::msg::Float64>(
                "/container_joint_cmd", 10);                

        rotor_cmd_pub_ =
             get_node()->create_publisher<std_msgs::msg::Float64>(
                "/rotor_joint_cmd", 10);
        vacuum_rotor_cmd_pub_ =
             get_node()->create_publisher<std_msgs::msg::Float64>(
                "/vacuum_rotor_joint_cmd", 10);
        brush_rotor_cmd_pub_ =
             get_node()->create_publisher<std_msgs::msg::Float64>(
                "/brush_rotor_joint_cmd", 10);
            joint_state_sub_ =
            get_node()->create_subscription<sensor_msgs::msg::JointState>(
                "/gz_joint_states",
              10,
              [this](const sensor_msgs::msg::JointState::SharedPtr msg)
              {
                  for (size_t i = 0; i < msg->name.size(); i++)
                  {
                      sim_positions_[msg->name[i]] = msg->position[i];
                      sim_vel_[msg->name[i]] = msg->velocity[i];
                  }
              });
        
        sampler_can_feedback_sub_ = get_node()->create_subscription<SamplerCanFeedback>(
            RosCanConstants::RosTopics::can_sampler_feedback, 10,
            std::bind(&SamplerHardware::feedbackCallback, this, std::placeholders::_1));

        sampler_can_cmd_pub_ = get_node()->create_publisher<SamplerCanCmd>(
            RosCanConstants::RosTopics::can_sampler_cmd,10);

        mRoverStatus_ = get_node()->create_subscription<rex_interfaces::msg::RoverStatus>(
            "/MQTT/RoverStatus",
            10,
            std::bind(&SamplerHardware::HandleRoverStatus, this, std::placeholders::_1)); 

        // timer_ = get_node()->create_wall_timer(
        // std::chrono::seconds(5),
        // [this]()
        // {
        //     std_msgs::msg::Float64 msg;
        //     msg.data = 0.4;
        //     platform_cmd_pub_->publish(msg);
        // });
    }

    

    return CallbackReturn::SUCCESS;
}

void SamplerHardware::feedbackCallback(const SamplerCanFeedback &feedback){
    for (size_t i = 0; i < feedback.actuator_id.size(); i++){
        real_positions_[feedback.actuator_id[i]] = feedback.positions[i];
        real_vel_[feedback.actuator_id[i]] = feedback.velocities[i];
    }
}

void SamplerHardware::HandleRoverStatus(const RoverStatusMsg::ConstSharedPtr &roverStatusMsg)
{

	LastStatusMsg = roverStatusMsg;

}

std::vector<hardware_interface::CommandInterface>
SamplerHardware::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    command_interfaces.emplace_back(
        "platform_joint",
        hardware_interface::HW_IF_POSITION,
        &platform_cmd_);
    
    command_interfaces.emplace_back(
        "platform_joint",
        hardware_interface::HW_IF_VELOCITY,
        &platform_vel_cmd_);

    command_interfaces.emplace_back(
        "drill_joint",
        hardware_interface::HW_IF_POSITION,
        &drill_cmd_);

    command_interfaces.emplace_back(
        "drill_joint",
        hardware_interface::HW_IF_VELOCITY,
        &drill_vel_cmd_);


    command_interfaces.emplace_back(
        "rotor_joint",
        hardware_interface::HW_IF_VELOCITY,
        &rotor_cmd_);
    command_interfaces.emplace_back(
        "vacuum_rotor_joint",
        hardware_interface::HW_IF_VELOCITY,
        &vacuum_rotor_cmd_);
    command_interfaces.emplace_back(
        "brush_rotor_joint",
        hardware_interface::HW_IF_VELOCITY,
        &brush_rotor_cmd_);
    
    command_interfaces.emplace_back(
    hardware_interface::CommandInterface(
        "container_joint",
        hardware_interface::HW_IF_POSITION,
        &container_cmd_));

    return command_interfaces;
}

std::vector<hardware_interface::StateInterface>
SamplerHardware::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;

    state_interfaces.emplace_back(
        "platform_joint",
        hardware_interface::HW_IF_POSITION,
        &platform_pos_);

    state_interfaces.emplace_back(
        "drill_joint",
        hardware_interface::HW_IF_POSITION,
        &drill_pos_);

    state_interfaces.emplace_back(
        "platform_joint",
        hardware_interface::HW_IF_VELOCITY,
        &platform_vel_);

    state_interfaces.emplace_back(
        "drill_joint",
        hardware_interface::HW_IF_VELOCITY,
        &drill_vel_);

    state_interfaces.emplace_back(
        "rotor_joint",
        hardware_interface::HW_IF_VELOCITY,
        &rotor_vel_);
    state_interfaces.emplace_back(
        "rotor_joint",
        hardware_interface::HW_IF_POSITION,
        &rotor_pos_);
    state_interfaces.emplace_back(
        "vacuum_rotor_joint",
        hardware_interface::HW_IF_VELOCITY,
        &vacuum_rotor_vel_);
    state_interfaces.emplace_back(
        "vacuum_rotor_joint",
        hardware_interface::HW_IF_POSITION,
        &vacuum_rotor_pos_);
    state_interfaces.emplace_back(
        "brush_rotor_joint",
        hardware_interface::HW_IF_VELOCITY,
        &brush_rotor_vel_);
     state_interfaces.emplace_back(
        "brush_rotor_joint",
        hardware_interface::HW_IF_POSITION,
        &brush_rotor_vel_);

    state_interfaces.emplace_back(
    hardware_interface::StateInterface(
        "container_joint",
        hardware_interface::HW_IF_POSITION,
        &container_pos_));

    state_interfaces.emplace_back(
    hardware_interface::StateInterface(
        "container_joint",
        hardware_interface::HW_IF_VELOCITY,
        &container_vel_));

    return state_interfaces;
}

hardware_interface::return_type SamplerHardware::read(const rclcpp::Time& /*time*/,
                                                            const rclcpp::Duration& /*period*/)
{

// Temporary: mirror command as feedback for testing
  if (sim_positions_.find("platform_joint") != sim_positions_.end())
  {
      platform_pos_ = sim_positions_["platform_joint"];
  }
  if (sim_positions_.find("drill_joint") != sim_positions_.end())
  {
      drill_pos_ = sim_positions_["drill_joint"];
  }
  if (sim_positions_.find("container_joint") != sim_positions_.end())
  {
      container_pos_ = sim_positions_["container_joint"];
  }

  if (sim_vel_.find("platform_joint") != sim_vel_.end())
  {
      platform_vel_ = sim_vel_["platform_joint"];
  }
  if (sim_vel_.find("drill_joint") != sim_vel_.end())
  {
      drill_vel_ = sim_vel_["drill_joint"];
  }
  if (sim_vel_.find("container_joint") != sim_vel_.end())
  {
      container_vel_ = sim_vel_["container_joint"];
  }
  if (sim_vel_.find("rotor_joint") != sim_vel_.end())
  {
    rotor_vel_ = sim_vel_["rotor_joint"];
  }
  if (sim_vel_.find("vacuum_rotor_joint") != sim_vel_.end())
  {
    vacuum_rotor_vel_ = sim_vel_["vacuum_rotor_joint"];
  }
  if (sim_vel_.find("brush_rotor_joint") != sim_vel_.end())
  {
    brush_rotor_vel_ = sim_vel_["brush_rotor_joint"];
  }


  if(real_positions_.find(RosCanConstants::VescIds::sampler_platform) != real_positions_.end()){
    platform_pos_ = real_positions_[RosCanConstants::VescIds::sampler_platform];
  }
  if(real_positions_.find(RosCanConstants::VescIds::sampler_drill_mov) != real_positions_.end()){
    drill_pos_ = real_positions_[RosCanConstants::VescIds::sampler_drill_mov];
  }
  if(real_positions_.find(RosCanConstants::VescIds::sampler_container_a) != real_positions_.end()){
    container_pos_ = real_positions_[RosCanConstants::VescIds::sampler_container_a];
  }
  if (real_vel_.find(RosCanConstants::VescIds::sampler_platform) != real_vel_.end())
  {
    platform_vel_ = real_vel_[RosCanConstants::VescIds::sampler_platform];
  }
  if(real_vel_.find(RosCanConstants::VescIds::sampler_drill_mov) != real_vel_.end()){
    drill_vel_ = real_vel_[RosCanConstants::VescIds::sampler_drill_mov];
  }
   if (real_vel_.find(RosCanConstants::VescIds::sampler_container_a) != real_vel_.end())
  {
    container_vel_ = real_vel_[RosCanConstants::VescIds::sampler_container_a];
  }
  if (real_vel_.find(RosCanConstants::VescIds::sampler_drill) != real_vel_.end())
  {
    rotor_vel_ = real_vel_[RosCanConstants::VescIds::sampler_drill];
  }
  if (real_vel_.find(RosCanConstants::VescIds::sampler_vacuum_suction) != real_vel_.end())
  {
    vacuum_rotor_vel_ = real_vel_[RosCanConstants::VescIds::sampler_vacuum_suction];
  }
  if (real_vel_.find(RosCanConstants::VescIds::sampler_vacuum_a) != real_vel_.end())
  {
    brush_rotor_vel_ = real_vel_[RosCanConstants::VescIds::sampler_vacuum_a];
  }
 

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type SamplerHardware::write(const rclcpp::Time& /*time*/,
                                                             const rclcpp::Duration& /*period*/)
{

  uint8_t command;
  if(LastStatusMsg->control_mode == RoverStatusMsg::CONTROL_MODE_AUTONOMY){
    command = VESC_COMMAND_SET_DUTY;
    sampler_mode = true;
   
  }
  else if(LastStatusMsg->control_mode == RoverStatusMsg::CONTROL_MODE_SAMPLER){
    command = VESC_COMMAND_SET_POS;
    sampler_mode = true;
    //set_pos = true;
  }else{
    command = VESC_COMMAND_SET_DUTY;
    sampler_mode = false;
  }
    cmd_.actuator_id[0] = RosCanConstants::VescIds::sampler_platform;
    cmd_.actuator_id[1] = RosCanConstants::VescIds::sampler_drill_mov;
    cmd_.actuator_id[2] = RosCanConstants::VescIds::sampler_container_a;
    cmd_.actuator_id[3] = RosCanConstants::VescIds::sampler_drill;
    cmd_.actuator_id[4] = RosCanConstants::VescIds::sampler_vacuum_suction;
    cmd_.actuator_id[5] = RosCanConstants::VescIds::sampler_vacuum_a;
    cmd_.actuator_id[6] = RosCanConstants::VescIds::sampler_vacuum_b;
  

  if (rclcpp::ok())
  {

    if(sampler_mode){
        cmd_.command_id[0] = command;
        cmd_.command_id[1] = command;
        cmd_.command_id[2] = command;
        cmd_.command_id[3] = VESC_COMMAND_SET_RPM;
        cmd_.command_id[4] = VESC_COMMAND_SET_RPM;
        cmd_.command_id[5] = VESC_COMMAND_SET_RPM;
        cmd_.command_id[6] = VESC_COMMAND_SET_RPM;


        cmd_.set_value[0] = platform_cmd_;
        cmd_.set_value[1] = drill_cmd_;
        cmd_.set_value[2] = container_cmd_;
        cmd_.set_value[3] = rotor_cmd_;
        cmd_.set_value[4] = vacuum_rotor_cmd_;
        cmd_.set_value[5] = brush_rotor_cmd_;
        cmd_.set_value[6] = 0.0;
    }else{
        
        cmd_.command_id[0] = VESC_COMMAND_SET_DUTY;
        cmd_.command_id[1] = VESC_COMMAND_SET_DUTY;
        cmd_.command_id[2] = VESC_COMMAND_SET_DUTY;
        cmd_.command_id[3] = VESC_COMMAND_SET_DUTY;
        cmd_.command_id[4] = VESC_COMMAND_SET_DUTY;
        cmd_.command_id[5] = VESC_COMMAND_SET_DUTY;
        cmd_.command_id[6] = VESC_COMMAND_SET_DUTY;


        cmd_.set_value[0] = 0.0;
        cmd_.set_value[1] = 0.0;
        cmd_.set_value[2] = 0.0;
        cmd_.set_value[3] = 0.0;
        cmd_.set_value[4] = 0.0;
        cmd_.set_value[5] = 0.0;
        cmd_.set_value[6] = 0.0;
    }
    
    sampler_can_cmd_pub_->publish(cmd_);

    //RCLCPP_INFO(get_node()->get_logger(), "Expected platform position: %f", last_platform_cmd_);
    std_msgs::msg::Float64 msg;
    msg.data = platform_cmd_;
    platform_cmd_pub_->publish(msg);

    msg.data = drill_cmd_;
    drill_cmd_pub_->publish(msg);

    msg.data = container_cmd_;
    container_cmd_pub_->publish(msg);

    msg.data = rotor_cmd_;
    rotor_cmd_pub_->publish(msg);

    msg.data = vacuum_rotor_cmd_;
    vacuum_rotor_cmd_pub_->publish(msg);

    msg.data = brush_rotor_cmd_;
    brush_rotor_cmd_pub_->publish(msg);
      //RCLCPP_INFO(get_node()->get_logger(), "Expected drill position: %f", last_drill_cmd_);
    //   msg.data = last_rotor_cmd_;
    //   rotor_cmd_pub_->publish(msg);
    }
   
  return hardware_interface::return_type::OK;
  }

  
}// end namespace sampler_hardware_interfaces

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(sampler_hardware_interfaces::SamplerHardware, hardware_interface::SystemInterface)