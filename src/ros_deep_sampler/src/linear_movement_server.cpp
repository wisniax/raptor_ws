
#include "ros_deep_sampler/linear_movement_server.hpp"

#include <functional>
#include <thread>
// #include "std_msgs/msg/Float64MultiArray.hpp"

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
      //(std::bind(&MoveLinearActionServer::execute, this, _1), goal_handle).detach());

    platform_pub_ =this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
        "/platform_controller/joint_trajectory", 10);
      
    drill_pub_ =this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
        "/drill_position_controller/joint_trajectory", 10);

        // Subscriber to joint states
    joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&MoveLinearActionServer::jointStateCallback, this, std::placeholders::_1));

    rotor_velocity_pub_ = this->create_publisher<std_msgs::msg::Float64>(
    "/rotor_velocity_controller/commands", 10);
  
  }



void MoveLinearActionServer::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
      for (size_t i = 0; i < msg->name.size(); ++i) {
        const std::string & name = msg->name[i];
        current_position_[name] = msg->position[i];
        current_velocity_[name] = msg->velocity[i];
    }
    
  }

double MoveLinearActionServer::get_current_position(int id){
  if(id == 1){
    return current_position_["platform_joint"];
  }
  if(id == 2){
    return current_position_["drill_joint"];
  }
  if(id == 3){
    return current_position_["rotor_joint"];
  }

}
double MoveLinearActionServer::get_current_velocity(int id){
  if(id == 1){
    return current_velocity_["platform_joint"];
  }
  if(id == 2){
    return current_velocity_["drill_joint"];
  }
  if(id == 3){
    return current_velocity_["rotor_joint"];
  }

}


  rclcpp_action::GoalResponse MoveLinearActionServer::handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const MoveLinearActionServer::Movement::Goal> goal)
  {
    // RCLCPP_INFO(this->get_logger(), "Received goal request with Id %d", goal->actuator_id);
    // RCLCPP_INFO(this->get_logger(), "Received goal request with position %f", goal->position);
    // RCLCPP_INFO(this->get_logger(), "Received goal request with velocity %f", goal->velocity);
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
    // const double actuator_id = goal->actuator_id;
    // const double target_position = goal->position;
    // //const double target_velocity = goal->velocity;
     const double tolerance = 0.005; // meters
    // const double time_to_position = target_position/goal->velocity;

    //RCLCPP_INFO(this->get_logger(), "Goal is %.3f", goal->position);
    rclcpp::Rate rate(50); // 50 Hz loop

    auto feedback = std::make_shared<Movement::Feedback>();
    auto result = std::make_shared<Movement::Result>();
    int current_slider_;

    trajectory_msgs::msg::JointTrajectory traj;

    for (const auto & cmd : goal->commands) {
        trajectory_msgs::msg::JointTrajectoryPoint point;
        double time_to_position =  (cmd.velocity > 0.001) ? std::fabs(cmd.position)/cmd.velocity : 1.0; 

        // Map actuator ID to joint name
        std::string joint_name;
        if (cmd.id == 1){ joint_name = "platform_joint";
                          current_slider_ = 1;}
        if (cmd.id == 2) {joint_name = "drill_joint";
                          current_slider_ =2;}
        if(cmd.id == 3) break;
        //if (cmd.id == 3) joint_name = "rotor_joint";
        
        traj.joint_names.push_back(joint_name);

        point.positions = {cmd.position};
        point.velocities = {0.0}; // or cmd.velocity if continuous joint
        traj.points.push_back(point);
        point.time_from_start = rclcpp::Duration::from_seconds(time_to_position);
    }
    if(current_slider_==1){
      platform_pub_->publish(traj);
    }
    else if(current_slider_ == 2){
      drill_pub_->publish(traj);
    }

    for (const auto & cmd : goal->commands) {
      if (cmd.id == 3) {  // rotor joint
          std_msgs::msg::Float64 rotor_msg;
          rotor_msg.data = cmd.velocity;
          rotor_velocity_pub_->publish(rotor_msg);
      }
  }

    while (rclcpp::ok()) {
        // Check if goal was canceled
        if (goal_handle->is_canceling()) {
            //result->final_position = current_slider_;
            goal_handle->canceled(result);
            // RCLCPP_INFO(this->get_logger(), "Goal canceled at %.3f", current_slider_);
            return;
        }


        // Publish feedback
        feedback->current_position.clear();
        feedback->current_velocity.clear();

        for (const auto & cmd : goal->commands) {
            // Replace these with actual readings from your hardware
            double pos = get_current_position(cmd.id);
            double vel = get_current_velocity(cmd.id);

            feedback->current_position.push_back(pos);
            feedback->current_velocity.push_back(vel);
        }

        goal_handle->publish_feedback(feedback);
        
        bool all_reached = true;
        for (const auto & cmd : goal->commands) {
          std::string joint_name;
          if (cmd.id == 1) joint_name = "platform_joint";
          else if (cmd.id == 2) joint_name = "drill_joint";
          else if (cmd.id == 3) joint_name = "rotor_joint";

          double pos = current_position_[joint_name];
          // optional: double vel = current_velocity_[joint_name];

          if (std::fabs(pos - cmd.position) > tolerance) {
              all_reached = false;
              break;  // one joint not reached, keep looping
          }
      }
        // Check if target reached
        if (all_reached) {
            result->success = true;
            goal_handle->succeed(result);
            //RCLCPP_INFO(this->get_logger(), "Goal succeeded! Reached %.3f", current_slider_);
            return;
        
        }
        
        rate.sleep();
    
    }
  }
 // class MoveLinearActionServer

}  // namespace action_tutorials_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(ros_deep_sampler::MoveLinearActionServer)