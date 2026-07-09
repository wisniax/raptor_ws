#include <memory>

#include "sampler_motion_interfaces/action/move_linear.hpp"
#include "sampler_motion_interfaces/msg/actuator_command.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <vector>
#include <unordered_map>
#include "ros_deep_sampler/visibility_control.h"



namespace ros_deep_sampler
{
class MoveLinearActionClient : public rclcpp::Node
{
public:
  using Movement = sampler_motion_interfaces::action::MoveLinear;
  using GoalHandleMovement = rclcpp_action::ClientGoalHandle<Movement>;

  explicit MoveLinearActionClient(const rclcpp::NodeOptions & options);


  void send_goal(const std::vector<sampler_motion_interfaces::msg::ActuatorCommand>  command,
                  bool calibrate_p = false, bool calibrate_d = false);
  int get_goal_status();
  void cancel_goal();
  void set_goal_status(bool status);
  void clear_feedback();
  double get_position(int id);
  double get_velocity(int id);

private:
  rclcpp_action::Client<Movement>::SharedPtr client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;
  GoalHandleMovement::SharedPtr current_goal_handle_;

  void goal_response_callback(GoalHandleMovement::SharedPtr goal_handle);

  void feedback_callback(
    GoalHandleMovement::SharedPtr,
    const std::shared_ptr<const Movement::Feedback> feedback);

  void result_callback(const GoalHandleMovement::WrappedResult & result);
  std::unordered_map<int, double> id_to_pos;
  std::unordered_map<int, double> id_to_vel;

  bool goal_completed;
  bool goal_canceled;
  double unknown_goal = -100.0;

};  // class MoveLinearActionClient

}