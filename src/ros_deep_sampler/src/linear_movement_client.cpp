#include "ros_deep_sampler/linear_movement_client.hpp"

#include <functional>
#include <thread>

#include "rclcpp_components/register_node_macro.hpp"

namespace ros_deep_sampler
{

MoveLinearActionClient::MoveLinearActionClient(const rclcpp::NodeOptions & options)
  : Node("Movement_action_client", options)
  {
    this->client_ptr_ = rclcpp_action::create_client<Movement>(
      this,
      "Movement");
    goal_completed = false;

    // this->timer_ = this->create_wall_timer(
    //   std::chrono::milliseconds(500),
    //   std::bind(&MoveLinearActionClient::send_goal, this));
  }


  void MoveLinearActionClient::send_goal(const std::vector<sampler_motion_interfaces::msg::ActuatorCommand>  commands)
  {
    using namespace std::placeholders;

    if (!this->client_ptr_->wait_for_action_server()) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      rclcpp::shutdown();
    }

    auto goal_msg = MoveLinearActionClient::Movement::Goal();
    goal_msg.commands = commands;

    RCLCPP_INFO(this->get_logger(), "Sending goal");

    auto send_goal_options = rclcpp_action::Client<Movement>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      std::bind(&MoveLinearActionClient::goal_response_callback, this, _1);
    send_goal_options.feedback_callback =
      std::bind(&MoveLinearActionClient::feedback_callback, this, _1, _2);
    send_goal_options.result_callback =
      std::bind(&MoveLinearActionClient::result_callback, this, _1);
    this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

  void MoveLinearActionClient::goal_response_callback(GoalHandleMovement::SharedPtr goal_handle)
  {
    //auto goal_handle = future.get();
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
  }

  void MoveLinearActionClient::feedback_callback(
    GoalHandleMovement::SharedPtr,
    const std::shared_ptr<const Movement::Feedback> feedback)
  {
    // for (size_t i = 0; i < feedback->current_position.size(); ++i) {
    //     RCLCPP_INFO(this->get_logger(),
    //                 "Actuator %zu: pos=%f, vel=%f",
    //                 i,
    //                 feedback->current_position[i],
    //                 feedback->current_velocity[i]);
    // }
    // RCLCPP_INFO(this->get_logger(), "Current position is: %f", feedback->current_position);
    // RCLCPP_INFO(this->get_logger(), "Current velocity is: %f", feedback->current_velocity);
 
  }

  void MoveLinearActionClient::result_callback(const GoalHandleMovement::WrappedResult & result)
  {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
        return;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        return;
    }
    RCLCPP_INFO(this->get_logger(), "Goal was reached");
    goal_completed = true;


    //rclcpp::shutdown();
  }  // class MoveLinearActionClient

  bool MoveLinearActionClient::get_goal_status(){
    return goal_completed;
  }

  void MoveLinearActionClient::set_goal_status(bool status){
    goal_completed = status;
  }

}  // namespace action_tutorials_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(ros_deep_sampler::MoveLinearActionClient)