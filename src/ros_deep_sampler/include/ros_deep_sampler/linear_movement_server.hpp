#include <memory>
#include "sampler_motion_interfaces/action/move_linear.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "ros_deep_sampler/visibility_control.h"

#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
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

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr slider_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;

  double current_slider_ = 0.0; 

};  // class MoveLinearActionServer

}  // namespace action_tutorials_cpp

