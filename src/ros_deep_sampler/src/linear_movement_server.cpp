
#include "ros_deep_sampler/linear_movement_server.hpp"

#include <functional>
#include <thread>

#include "rclcpp_components/register_node_macro.hpp"

namespace ros_deep_sampler
{


 MoveLinearActionServer::MoveLinearActionServer(const rclcpp::NodeOptions & options)
  : Node("move_linear", options)
  {
    using namespace std::placeholders;

    this->action_server_ = rclcpp_action::create_server<Movement>(
      this,
      "movement",
      std::bind(&MoveLinearActionServer::handle_goal, this, _1, _2),
      std::bind(&MoveLinearActionServer::handle_cancel, this, _1),
      std::bind(&MoveLinearActionServer::handle_accepted, this, _1));
  }




  rclcpp_action::GoalResponse MoveLinearActionServer::handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const MoveLinearActionServer::Movement::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal request with Id %d", goal->actuator_id);
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse MoveLinearActionServer::handle_cancel(
    const std::shared_ptr<GoalHandleMovement> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void MoveLinearActionServer::handle_accepted(const std::shared_ptr<GoalHandleMovement> goal_handle)
  {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&MoveLinearActionServer::execute, this, _1), goal_handle}.detach();
  }

  void MoveLinearActionServer::execute(const std::shared_ptr<GoalHandleMovement> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    rclcpp::Rate loop_rate(1);
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<Movement::Feedback>();
    auto & current_position = feedback->current_position;
    auto & current_velocity = feedback->current_velocity;
    auto & state = feedback->state;
    // sequence.push_back(0);
    // sequence.push_back(1);
    auto result = std::make_shared<Movement::Result>();

    

    // for (int i = 1; (i < goal->order) && rclcpp::ok(); ++i) {
    //   // Check if there is a cancel request
    //   if (goal_handle->is_canceling()) {
    //     result->sequence = sequence;
    //     goal_handle->canceled(result);
    //     RCLCPP_INFO(this->get_logger(), "Goal canceled");
    //     return;
    //   }
    //   // Update sequence
    //   sequence.push_back(sequence[i] + sequence[i - 1]);
    //   // Publish feedback
    //   goal_handle->publish_feedback(feedback);
    //   RCLCPP_INFO(this->get_logger(), "Publish feedback");

    //   loop_rate.sleep();
    // }

    // Check if goal is done
    if (rclcpp::ok()) {
      result->success = true;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }
  }
 // class MoveLinearActionServer

}  // namespace action_tutorials_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(ros_deep_sampler::MoveLinearActionServer)