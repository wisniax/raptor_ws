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

  // // msg.set_data(1.0);

  // // bool success = pub_platform.Publish(msg);

  // // for (int i = 0; i < 50; ++i)
  // // {
  // //     pub_platform.Publish(msg);
  // //     std::this_thread::sleep_for(std::chrono::milliseconds(100));
  // // }

  //  if (!success)
  // {
  //   RCLCPP_ERROR(get_node()->get_logger(), "Publishing to joint failed");
  //   return CallbackReturn::ERROR;
  // }else{
  //   RCLCPP_INFO(get_node()->get_logger(), "Successfuly published to joint");
  // }

  



//   topic_based_joint_commands_publisher_ = get_node()->create_publisher<sensor_msgs::msg::JointState>(
//       get_hardware_parameter("joint_commands_topic", "/robot_joint_commands"), rclcpp::QoS(1));
//   topic_based_joint_states_subscriber_ = get_node()->create_subscription<sensor_msgs::msg::JointState>(
//       get_hardware_parameter("joint_states_topic", "/robot_joint_states"), rclcpp::SensorDataQoS(),
//       [this](const sensor_msgs::msg::JointState::SharedPtr joint_state) { latest_joint_state_ = *joint_state; });

  
  

  // if the values on the `joint_states_topic` are wrapped between -2*pi and 2*pi (like they are in Isaac Sim)
  // sum the total joint rotation returned on the `joint_state_values_` interface


  // TODO(anyone): Remove in a future release after users have migrated to the new plugin name
//   if (get_hardware_info().hardware_plugin_name.find("topic_based_ros2_control") != std::string::npos)
//   {
//     RCLCPP_WARN(get_node()->get_logger(),
//                 "Plugin name '%s' is deprecated, upgrade to "
//                 "'joint_state_topic_hardware_interface/JointStateTopicSystem' from package"
//                 " 'joint_state_topic_hardware_interface' instead.",
//                 get_hardware_info().hardware_plugin_name.c_str());
//   }

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

        // rotor_cmd_pub_ =
        //     get_node()->create_publisher<std_msgs::msg::Float64>(
        //         "/rotor_joint_cmd", 10);

        // timer_ = get_node()->create_wall_timer(
        //     std::chrono::seconds(5),
        //     [this]()
        //     {
        //         std_msgs::msg::Float64 msg;
        //         msg.data = 0.4;   // desired joint position
        //         rotor_cmd_pub_->publish(msg);
        //     });
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
        "rotor_joint",
        hardware_interface::HW_IF_POSITION,
        &rotor_pos_);

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
      // RCLCPP_INFO(
      //   get_node()->get_logger(),
      //   "READ platform: state=%f command=%f",
      //   platform_pos_,
      //   platform_cmd_);
  }

  if (sim_positions_.find("drill_joint") != sim_positions_.end())
  {
      drill_pos_ = sim_positions_["drill_joint"];
  }

  if (sim_positions_.find("rotor_joint") != sim_positions_.end())
  {
      rotor_pos_ = sim_positions_["rotor_joint"];
  }

   if (sim_positions_.find("container_joint") != sim_positions_.end())
  {
      container_pos_ = sim_positions_["container_joint"];
  }

  if (sim_positions_.find("platform_joint") != sim_positions_.end())
  {
      platform_vel_ = sim_positions_["platform_joint"];
      // RCLCPP_INFO(
      //   get_node()->get_logger(),
      //   "READ platform: state=%f command=%f",
      //   platform_pos_,
      //   platform_cmd_);
  }

  if (sim_vel_.find("drill_joint") != sim_vel_.end())
  {
      drill_vel_ = sim_vel_["drill_joint"];
  }

  if (sim_vel_.find("rotor_joint") != sim_vel_.end())
  {
      rotor_vel_ = sim_vel_["rotor_joint"];
  }

   if (sim_vel_.find("container_joint") != sim_vel_.end())
  {
      container_vel_ = sim_vel_["container_joint"];
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type SamplerHardware::write(const rclcpp::Time& /*time*/,
                                                             const rclcpp::Duration& /*period*/)
{
  if(!(std::isnan(platform_cmd_))){
    // if(!(std::isnan(last_platform_cmd_)))
    last_platform_cmd_ = platform_cmd_;
  }

  if(!(std::isnan(drill_cmd_))){
    // if(!(std::isnan(last_platform_cmd_)))
    last_drill_cmd_ = drill_cmd_;
  }
  if(!(std::isnan(container_cmd_))){
    // if(!(std::isnan(last_platform_cmd_)))
    last_container_cmd_ = container_cmd_;
  }

//   if(!(std::isnan(rotor_cmd_))){
//     // if(!(std::isnan(last_platform_cmd_)))
//     last_rotor_cmd_ = rotor_cmd_;
//   }
  // last_platform_cmd_ += 0.01;
  //RCLCPP_INFO(get_node()->get_logger(), "Expected current position: %f", last_platform_cmd_);

  

  // 2. Send command to hardware (CAN / MCU / simulation)
  

  if (rclcpp::ok())
  {
      std_msgs::msg::Float64 msg;
      msg.data = last_platform_cmd_;
      //RCLCPP_INFO(get_node()->get_logger(), "Expected platform position: %f", last_platform_cmd_);
      platform_cmd_pub_->publish(msg);

      msg.data = last_drill_cmd_;
      drill_cmd_pub_->publish(msg);

      msg.data = last_container_cmd_;
      container_cmd_pub_->publish(msg);
      //RCLCPP_INFO(get_node()->get_logger(), "Expected drill position: %f", last_drill_cmd_);
    //   msg.data = last_rotor_cmd_;
    //   rotor_cmd_pub_->publish(msg);
    }
   
  return hardware_interface::return_type::OK;
  }

  
}// end namespace sampler_hardware_interfaces

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(sampler_hardware_interfaces::SamplerHardware, hardware_interface::SystemInterface)