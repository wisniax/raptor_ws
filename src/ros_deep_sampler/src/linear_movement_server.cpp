
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
      "Movement",
      std::bind(&MoveLinearActionServer::handle_goal, this, _1, _2),
      std::bind(&MoveLinearActionServer::handle_cancel, this, _1),
      std::bind(&MoveLinearActionServer::handle_accepted, this, _1));

     slider_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/slider_position_controller/commands", 10);

        // Subscriber to joint states
    joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&MoveLinearActionServer::jointStateCallback, this, std::placeholders::_1));
  
  }



void MoveLinearActionServer::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        // Update current slider position
        for (size_t i = 0; i < msg->name.size(); ++i) {
            if (msg->name[i] == "slider_joint") {
                current_slider_ = msg->position[i];
                break;
            }
        }
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

    const auto goal = goal_handle->get_goal();
    const double actuator_id = goal->actuator_id;
    const double target_position = goal->position;
    const double tolerance = 0.005; // meters
    rclcpp::Rate rate(50); // 50 Hz loop

    auto feedback = std::make_shared<Movement::Feedback>();
    auto result = std::make_shared<Movement::Result>();

    while (rclcpp::ok()) {
        // Check if goal was canceled
        if (goal_handle->is_canceling()) {
            //result->final_position = current_slider_;
            goal_handle->canceled(result);
            RCLCPP_INFO(this->get_logger(), "Goal canceled at %.3f", current_slider_);
            return;
        }

        // Publish command to controller
        std_msgs::msg::Float64MultiArray cmd_msg;
        if(actuator_id == 1){
          cmd_msg.data = {target_position};
        }
        slider_pub_->publish(cmd_msg);

        // Publish feedback
        feedback->current_position = current_slider_;
        goal_handle->publish_feedback(feedback);

        // Check if target reached
        if (std::fabs(current_slider_ - target_position) < tolerance) {
            result->success = true;
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "Goal succeeded! Reached %.3f", current_slider_);
            return;
        }

        rate.sleep();
    
    }
  }
 // class MoveLinearActionServer

}  // namespace action_tutorials_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(ros_deep_sampler::MoveLinearActionServer)