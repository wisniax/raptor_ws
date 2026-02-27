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

    // this->timer_ = this->create_wall_timer(
    //   std::chrono::milliseconds(500),
    //   std::bind(&MoveLinearActionClient::send_goal, this));
  }

  void MoveLinearActionClient::send_goal()
  {
    using namespace std::placeholders;

    //this->timer_->cancel();

    if (!this->client_ptr_->wait_for_action_server()) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      rclcpp::shutdown();
    }

    auto goal_msg = MoveLinearActionClient::Movement::Goal();
    goal_msg.actuator_id = 1;
    goal_msg.position = 0.15;
    goal_msg.velocity = 0.05;

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
    std::stringstream ss;
    // ss << "Result received: ";
    // for (auto number : result.result->sequence) {
    //   ss << number << " ";
    // }
    // RCLCPP_INFO(this->get_logger(), ss.str().c_str());
    rclcpp::shutdown();
  }  // class MoveLinearActionClient

}  // namespace action_tutorials_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(ros_deep_sampler::MoveLinearActionClient)