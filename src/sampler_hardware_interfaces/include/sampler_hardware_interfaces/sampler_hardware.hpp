#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include "rex_interfaces/msg/rover_status.hpp"
#include <sensor_msgs/msg/joint_state.hpp>
#include "sampler_motion_interfaces/msg/sampler_can_cmd.hpp"
#include "sampler_motion_interfaces/msg/sampler_can_feedback.hpp"
#include <gz/transport/Node.hh>
#include <gz/msgs/double.pb.h>
#include <iostream>
#include "std_msgs/msg/float64.hpp"
#include "sampler_hardware_interfaces/RosCanConstants.hpp"


namespace sampler_hardware_interfaces
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using SamplerCanCmd = sampler_motion_interfaces::msg::SamplerCanCmd;
using SamplerCanFeedback = sampler_motion_interfaces::msg::SamplerCanFeedback;
using RoverStatusMsg = rex_interfaces::msg::RoverStatus;


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

  void feedbackCallback(const SamplerCanFeedback &feedback);
  void HandleRoverStatus(const RoverStatusMsg::ConstSharedPtr &roverStatusMsg);


private:
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joints_feedback_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr topic_based_joint_commands_publisher_;
  sensor_msgs::msg::JointState latest_joint_state_;
  bool sum_wrapped_joint_states_{ false };

  // If the difference between the current joint state and joint command is less than this value,
  // the joint command will not be published.
  double trigger_joint_command_threshold_ = 1e-5;

  std::unordered_map<std::string, uint8_t> joint_to_can_id_;


  double platform_cmd_ = 0.0;
  double platform_vel_cmd_ = 0.0;
  double drill_cmd_ = 0.0;
  double drill_vel_cmd_ = 0.0;
  double container_cmd_ = 0.0;
  double rotor_cmd_ = 0.0;
  double vacuum_rotor_cmd_ = 0.0;
  double brush_rotor_cmd_ =0.0;

  // double last_platform_cmd_ =0.0;
  // double last_drill_cmd_ = 0.0;
  // double last_rotor_cmd_ =0.0;
  // double last_container_cmd_ = 0.0;

  double platform_pos_ = 0.0;
  double drill_pos_ = 0.0;
  double rotor_pos_ = 0.0;
  double vacuum_rotor_pos_ = 0.0;
  double brush_rotor_pos_ = 0.0;
  double platform_vel_ = 0.0;
  double drill_vel_ = 0.0;
  double rotor_vel_ = 0.0;
  double vacuum_rotor_vel_ = 0.0;
  double brush_rotor_vel_ = 0.0;
  double container_pos_ = 0.0;
  double container_vel_ = 0.0;


  rclcpp::Publisher<SamplerCanCmd>::SharedPtr sampler_can_cmd_pub_;
  rclcpp::Subscription<SamplerCanFeedback>::SharedPtr sampler_can_feedback_sub_;
  rclcpp::Subscription<rex_interfaces::msg::RoverStatus>::SharedPtr mRoverStatus_;

  rclcpp::TimerBase::SharedPtr timer_;


  std::unordered_map<uint8_t, double> real_positions_;
  std::unordered_map<uint8_t, double> real_vel_;

  SamplerCanCmd cmd_;
  SamplerCanFeedback::SharedPtr feedback_; 
  RoverStatusMsg::ConstSharedPtr LastStatusMsg;

  bool sampler_mode = false;
  bool set_pos = false;

  // gz::transport::Node node;
  // gz::msgs::Double msg;
  // auto pub_platform = node.create_publisher<std_msgs::msg::Float64>(
  //           "/platform_joint_cmd", 10);
  // auto pub_drill = node.Advertise<gz::msgs::Double>(
  //     "/model/sampler/joint/drill_joint/0/cmd_pos");
};

}  // namespace sampler_hardware_interfaces