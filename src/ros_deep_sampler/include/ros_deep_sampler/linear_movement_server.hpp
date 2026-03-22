#include <memory>
#include "sampler_motion_interfaces/action/move_linear.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "ros_deep_sampler/visibility_control.h"

#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <unordered_map>
#include <string>

using namespace std::chrono;

namespace ros_deep_sampler
{
class MoveLinearActionServer : public rclcpp::Node
{
public:
  using Movement = sampler_motion_interfaces::action::MoveLinear;
  using GoalHandleMovement = rclcpp_action::ServerGoalHandle<Movement>;

  //ACTION_TUTORIALS_CPP_PUBLIC
  explicit MoveLinearActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  double get_current_position(int id);
  double get_current_velocity(int id);


private:
  rclcpp_action::Server<Movement>::SharedPtr action_server_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Movement::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleMovement> goal_handle);
  
  void handle_accepted(const std::shared_ptr<GoalHandleMovement> goal_handle);
  

  void execute(const std::shared_ptr<GoalHandleMovement> goal_handle);
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);

  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;

  std::unordered_map<std::string, double> current_position_;
  std::unordered_map<std::string, double> current_velocity_;

};  // class MoveLinearActionServer

}  // namespace action_tutorials_cpp

