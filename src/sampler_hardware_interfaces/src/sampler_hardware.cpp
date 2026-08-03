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

  return CallbackReturn::SUCCESS;
}

void SamplerHardware::publish_platform_cmd(){


}

CallbackReturn SamplerHardware::on_configure(
    const rclcpp_lifecycle::State &)
{
    if (get_node())
<<<<<<< HEAD
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
=======
    {   
        sampler_can_feedback_sub_ = get_node()->create_subscription<SamplerCanFeedback>(
            RosCanConstants::RosTopics::can_sampler_feedback, 10,
            std::bind(&SamplerHardware::feedbackCallback, this, std::placeholders::_1));

        sampler_can_cmd_pub_ = get_node()->create_publisher<SamplerCanCmd>(
            RosCanConstants::RosTopics::can_sampler_cmd,10);

        mRoverStatus_ = get_node()->create_subscription<rex_interfaces::msg::RoverStatus>(
            "/MQTT/RoverStatus",
            10,
            std::bind(&SamplerHardware::HandleRoverStatus, this, std::placeholders::_1)); 

>>>>>>> real-hardware
    }

    

    return CallbackReturn::SUCCESS;
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


  return hardware_interface::return_type::OK;
}

hardware_interface::return_type SamplerHardware::write(const rclcpp::Time& /*time*/,
                                                             const rclcpp::Duration& /*period*/)
{

<<<<<<< HEAD
=======
  // uint8_t command;
  // if(LastStatusMsg->control_mode == RoverStatusMsg::CONTROL_MODE_AUTONOMY){
  //   command = VESC_COMMAND_SET_DUTY;
  //   sampler_mode = true;
   
  // }
  // else if(LastStatusMsg->control_mode == RoverStatusMsg::CONTROL_MODE_SAMPLER){
  //   command = VESC_COMMAND_SET_POS;
  //   sampler_mode = true;
  //   //set_pos = true;
  // }else{
  //   command = VESC_COMMAND_SET_DUTY;
  //   sampler_mode = false;
  // }
    cmd_.actuator_id[0] = RosCanConstants::VescIds::sampler_platform;
    cmd_.actuator_id[1] = RosCanConstants::VescIds::sampler_drill_mov;
    cmd_.actuator_id[2] = RosCanConstants::VescIds::sampler_container_a;
    cmd_.actuator_id[3] = RosCanConstants::VescIds::sampler_drill;
    cmd_.actuator_id[4] = RosCanConstants::VescIds::sampler_vacuum_suction;
    cmd_.actuator_id[5] = RosCanConstants::VescIds::sampler_vacuum_a;
    cmd_.actuator_id[6] = RosCanConstants::VescIds::sampler_vacuum_b;

    cmd_.command_id[0] = pos_command;
    cmd_.command_id[1] = pos_command;
    cmd_.command_id[2] = pos_command;
    cmd_.command_id[3] = rpm_command;
    cmd_.command_id[4] = rpm_command;
    cmd_.command_id[5] = rpm_command;
    cmd_.command_id[6] = rpm_command;


    cmd_.set_value[0] = platform_cmd_;
    cmd_.set_value[1] = drill_cmd_;
    cmd_.set_value[2] = container_cmd_;
    cmd_.set_value[3] = rotor_cmd_;
    cmd_.set_value[4] = vacuum_rotor_cmd_;
    cmd_.set_value[5] = brush_rotor_cmd_;
    cmd_.set_value[6] = 0.0;
>>>>>>> real-hardware

  if (rclcpp::ok())
  {

<<<<<<< HEAD
=======
    // if(sampler_mode){
       
    // }else{
        
    //     cmd_.command_id[0] = VESC_COMMAND_SET_DUTY;
    //     cmd_.command_id[1] = VESC_COMMAND_SET_DUTY;
    //     cmd_.command_id[2] = VESC_COMMAND_SET_DUTY;
    //     cmd_.command_id[3] = VESC_COMMAND_SET_DUTY;
    //     cmd_.command_id[4] = VESC_COMMAND_SET_DUTY;
    //     cmd_.command_id[5] = VESC_COMMAND_SET_DUTY;
    //     cmd_.command_id[6] = VESC_COMMAND_SET_DUTY;


    //     cmd_.set_value[0] = 0.0;
    //     cmd_.set_value[1] = 0.0;
    //     cmd_.set_value[2] = 0.0;
    //     cmd_.set_value[3] = 0.0;
    //     cmd_.set_value[4] = 0.0;
    //     cmd_.set_value[5] = 0.0;
    //     cmd_.set_value[6] = 0.0;
    // }
    
    sampler_can_cmd_pub_->publish(cmd_);
>>>>>>> real-hardware

    }
   
  return hardware_interface::return_type::OK;
  }

  
}// end namespace sampler_hardware_interfaces

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(sampler_hardware_interfaces::SamplerHardware, hardware_interface::SystemInterface)