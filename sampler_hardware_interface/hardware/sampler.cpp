#include "sampler_hardware/sampler.hpp"

#include <chrono>
#include <cmath>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"


namespace ros_deep_sampler{

hardware_interface::CallbackReturn SamplerHardware::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (
    hardware_interface::SystemInterface::on_init(params) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  node_ = rclcpp::Node::make_shared("sampler_hardware");

  SamplerCtrlPub_ =
    node_->create_publisher<rex_interfaces::msg::SamplerControl>(
        "/MQTT/SamplerControl", 10);

//   // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
//   hw_start_sec_ = stod(info_.hardware_parameters["example_param_hw_start_duration_sec"]);
//   hw_stop_sec_ = stod(info_.hardware_parameters["example_param_hw_stop_duration_sec"]);
//   hw_slowdown_ = stod(info_.hardware_parameters["example_param_hw_slowdown"]);
//   RCLCPP_INFO(get_logger(), "Robot hardware_component update_rate is %dHz", info_.rw_rate);
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // RRBotSystemPositionOnly has exactly one state and command interface on each joint
    if (joint.command_interfaces.empty())
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s'no command interfaces!",
        joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.empty()) {
            RCLCPP_FATAL(get_logger(), "Joint '%s' has no state interfaces!", joint.name.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }
    if (joint.name == "slider_joint" || joint.name == "drill_joint") {
            if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
                RCLCPP_FATAL(get_logger(),
                    "Joint '%s' expected POSITION command interface", joint.name.c_str());
                return hardware_interface::CallbackReturn::ERROR;
            }
        } else if (joint.name == "rotor_joint") {
            if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY) {
                RCLCPP_FATAL(get_logger(),
                    "Joint '%s' expected VELOCITY command interface", joint.name.c_str());
                return hardware_interface::CallbackReturn::ERROR;
            }
        } else {
            RCLCPP_WARN(get_logger(), "Unexpected joint '%s' found", joint.name.c_str());
        }
    }

    RCLCPP_INFO(get_logger(), "All joints are correctly defined!");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SamplerHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
//   RCLCPP_INFO(get_logger(), "Configuring ...please wait...");

//   for (int i = 0; i < hw_start_sec_; i++)
//   {
//     rclcpp::sleep_for(std::chrono::seconds(1));
//     RCLCPP_INFO(get_logger(), "%.1f seconds left...", hw_start_sec_ - i);
//   }
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  // reset values always when configuring hardware
  for (const auto & [name, descr] : joint_state_interfaces_)
  {
    set_state(name, 0.0);
  }
  for (const auto & [name, descr] : joint_command_interfaces_)
  {
    set_command(name, 0.0);
  }
  RCLCPP_INFO(get_logger(), "Successfully configured!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SamplerHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
//   // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
//   RCLCPP_INFO(get_logger(), "Activating ...please wait...");

//   for (int i = 0; i < hw_start_sec_; i++)
//   {
//     rclcpp::sleep_for(std::chrono::seconds(1));
//     RCLCPP_INFO(get_logger(), "%.1f seconds left...", hw_start_sec_ - i);
//   }
//   // END: This part here is for exemplary purposes - Please do not copy to your production code

//   // command and state should be equal when starting
//   for (const auto & [name, descr] : joint_state_interfaces_)
//   {
//     set_command(name, get_state(name));
//   }

//   RCLCPP_INFO(get_logger(), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SamplerHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
//   // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
//   RCLCPP_INFO(get_logger(), "Deactivating ...please wait...");

//   for (int i = 0; i < hw_stop_sec_; i++)
//   {
//     rclcpp::sleep_for(std::chrono::seconds(1));
//     RCLCPP_INFO(get_logger(), "%.1f seconds left...", hw_stop_sec_ - i);
//   }

//   RCLCPP_INFO(get_logger(), "Successfully deactivated!");
//   // END: This part here is for exemplary purposes - Please do not copy to your production code

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type SamplerHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
   rclcpp::spin_some(node_);

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type SamplerHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  rex_interfaces::msg::SamplerControl  cmd;
  double platform_cmd = get_command("platform_joint");  // <-- controller wrote this
  double drill_cmd = get_command("drill_joint");
  double rotor_cmd = get_command("rotor_joint");

    // Now you send it to your motor driver
    cmd.platform_movement = platform_cmd;
    cmd.drill_movement = drill_cmd;
    cmd.drill_action = rotor_cmd;
    
    SamplerCtrlPub_->publish(cmd);

    return hardware_interface::return_type::OK;
  
}

}  // namespace ros2_control_demo_example_1


