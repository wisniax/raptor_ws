#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>

#include <sensor_msgs/msg/joint_state.hpp>

#include <gz/transport/Node.hh>
#include <gz/msgs/double.pb.h>
#include <iostream>
#include "std_msgs/msg/float64.hpp"

namespace sampler_hardware_interfaces
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class SamplerHardware : public hardware_interface::SystemInterface
{
public:
  CallbackReturn on_init(const hardware_interface::HardwareComponentInterfaceParams& params) override;

   CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;

  hardware_interface::return_type write(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces();
  std::vector<hardware_interface::StateInterface>  export_state_interfaces();

  void publish_platform_cmd();

private:
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joints_feedback_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr topic_based_joint_commands_publisher_;
  sensor_msgs::msg::JointState latest_joint_state_;
  bool sum_wrapped_joint_states_{ false };

  // If the difference between the current joint state and joint command is less than this value,
  // the joint command will not be published.
  double trigger_joint_command_threshold_ = 1e-5;

  std::unordered_map<std::string, uint8_t> joint_to_can_id_;
  double platform_cmd_;
  double platform_vel_cmd_;
  double drill_cmd_;
  double drill_vel_cmd_;
  double rotor_cmd_;

  double last_platform_cmd_ =0.0;
  double last_drill_cmd_ = 0.0;
  double last_rotor_cmd_ =0.0;
  double last_container_cmd_ = 0.0;

  double platform_pos_ = 0.0;
  double drill_pos_ = 0.0;
  double rotor_pos_ = 0.0;

  double platform_vel_ = 0.0;
  double drill_vel_ = 0.0;
  double rotor_vel_ = 0.0;
  double vacuum_rotor_vel_ = 0.0;
  double brush_rotor_vel_ = 0.0;

  double container_pos_ = 0.0;
  double container_vel_ = 0.0;

  double container_cmd_ = 0.0;

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr platform_cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr drill_cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr rotor_cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr container_cmd_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::unordered_map<std::string, double> sim_positions_;
  std::unordered_map<std::string, double> sim_vel_;

  // gz::transport::Node node;
  // gz::msgs::Double msg;
  // auto pub_platform = node.create_publisher<std_msgs::msg::Float64>(
  //           "/platform_joint_cmd", 10);
  // auto pub_drill = node.Advertise<gz::msgs::Double>(
  //     "/model/sampler/joint/drill_joint/0/cmd_pos");
};

}  // namespace sampler_hardware_interfaces