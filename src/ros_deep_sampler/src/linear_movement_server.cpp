
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

    rotor_velocity_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
    "/rotor_velocity_controller/commands", 10);
  
  }



void MoveLinearActionServer::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
      // RCLCPP_INFO(this->get_logger(), "Received State Callback");
      for (size_t i = 0; i < msg->name.size(); ++i) {
        const std::string & name = msg->name[i];
        current_position_[name] = msg->position[i];
        current_velocity_[name] = msg->velocity[i];
    }
    
  }

double MoveLinearActionServer::get_current_position(int id){
  if(id == RosCanConstants::VescIds::sampler_platform){
    return current_position_["platform_joint"];
  }
  if(id == RosCanConstants::VescIds::sampler_drill_mov){
    return current_position_["drill_joint"];
  }
  if(id == RosCanConstants::VescIds::sampler_drill){
    return current_position_["rotor_joint"];
  }

}
double MoveLinearActionServer::get_current_velocity(int id){
  if(id == RosCanConstants::VescIds::sampler_platform){
    return current_velocity_["platform_joint"];
  }
  if(id == RosCanConstants::VescIds::sampler_drill_mov){
    return current_velocity_["drill_joint"];
  }
  if(id == RosCanConstants::VescIds::sampler_drill){
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
    const double tolerance = 0.001; // meters
    // const double time_to_position = target_position/goal->velocity;

    //RCLCPP_INFO(this->get_logger(), "Goal is %.3f", goal->position);
    rclcpp::Rate rate(50); // 50 Hz loop

    auto feedback = std::make_shared<Movement::Feedback>();
    auto result = std::make_shared<Movement::Result>();
    int current_slider_;
    float current_slider_pos;
    float current_slider_vel;
    int current_id;
    float final_pos;
    double dist;
    trajectory_msgs::msg::JointTrajectory traj;
    trajectory_msgs::msg::JointTrajectoryPoint point;
    std::string joint_name;
    for(const auto &cmd: goal->commands){
      
        if (cmd.id == RosCanConstants::VescIds::sampler_platform){ joint_name = "platform_joint";
            // if(cmd.position == -200.0 && cmd.velocity == -200.0){
            //   RCLCPP_INFO(this->get_logger(), "Initial calibration");
            //   feedback->current_position.clear();
            //   double pos = get_current_position(cmd.id);
            //   double vel = get_current_velocity(cmd.id);

            //   feedback->ids.push_back(cmd.id);
            //   feedback->current_position.push_back(pos);
            //   feedback->current_velocity.push_back(vel);
            //   RCLCPP_INFO(this->get_logger(), "Current pos: %f", pos);
            //   RCLCPP_INFO(this->get_logger(), "Current vel: %f", vel);
            //   goal_handle->publish_feedback(feedback);
            //   result->success = true;
            //   goal_handle->succeed(result);
            //   RCLCPP_INFO(this->get_logger(), "Initial pose of platform send");
            //   return;
       
            // }
            current_slider_ = 1;
            current_id = cmd.id;
            dist = cmd.position;
            if (goal->calibrate_platform){
              final_pos = 0.0;
            }else{
              final_pos = dist + get_current_position(cmd.id);
            }
            current_slider_pos = get_current_position(current_id);
            current_slider_vel = cmd.velocity;
          }
        if (cmd.id == RosCanConstants::VescIds::sampler_drill_mov) {joint_name = "drill_joint";
            current_slider_ = 2;
            current_id = cmd.id;
            dist = cmd.position;
            if (goal->calibrate_drill){
              final_pos = 0.0;
            }else{
              final_pos = dist + get_current_position(cmd.id);
            }
            current_slider_pos = get_current_position(current_id);
            current_slider_vel = cmd.velocity;
          }
       
        if(cmd.id == RosCanConstants::VescIds::sampler_drill) break;
    }
    
    
    // if(current_slider_ == 1){
    //   dist = final_pos;
    //   RCLCPP_INFO(this->get_logger(), "Distance to position: %f", dist);
    //    //prev_platform_pos = current_slider_pos;
    // }
    // if(current_slider_ == 2){
    //    dist = final_pos;
    //    //prev_drill_pos = current_slider_pos;
    // }
    double time_to_position =  (current_slider_vel >= 0.001) ? std::fabs(dist)/current_slider_vel : 1.0;  
    RCLCPP_INFO(this->get_logger(), "Time to position: %f",time_to_position);
    point.positions = {dist};
    point.time_from_start = rclcpp::Duration::from_seconds(time_to_position);
    traj.joint_names.push_back(joint_name);
    
    
    //point.velocities = {0.0}; // or cmd.velocity if continuous joint
    
    traj.points.push_back(point);

    //RCLCPP_INFO(this->get_logger(), "Size of vector commands: %.3d", (goal->commands).size());
    if(current_slider_==1){
      platform_pub_->publish(traj);
    }
    else if(current_slider_ == 2){
      drill_pub_->publish(traj);
    }

    for (const auto & cmd : goal->commands) {
      if (cmd.id == RosCanConstants::VescIds::sampler_drill) {  // rotor joint
        auto rotor_msg = std_msgs::msg::Float64MultiArray();
        rotor_msg.data = {cmd.velocity};  // one joint → one value
          rotor_velocity_pub_->publish(rotor_msg);
      }
    }

    while (rclcpp::ok()) {
        // Check if goal was canceled
       if (goal_handle->is_canceling()) {

          trajectory_msgs::msg::JointTrajectory traj;

          traj.joint_names = {joint_name};

          trajectory_msgs::msg::JointTrajectoryPoint point;

          point.positions = {get_current_position(current_id)};
          point.velocities = {0.0};
          point.time_from_start = rclcpp::Duration::from_seconds(0.1);

          traj.points.push_back(point);

          if (current_slider_ == 1) {
              platform_pub_->publish(traj);
          } else if (current_slider_ == 2) {
              drill_pub_->publish(traj);
          }

          auto rotor_msg = std_msgs::msg::Float64MultiArray();
          rotor_msg.data = {0.0};
          rotor_velocity_pub_->publish(rotor_msg);

          RCLCPP_INFO(this->get_logger(), "Goal canceled at %d", current_slider_);

          goal_handle->canceled(result);
          return;
        }


        // Publish feedback
        feedback->current_position.clear();
        feedback->current_velocity.clear();

        // for (const auto & cmd : goal->commands) {
        //     // Replace these with actual readings from your hardware
        //     double pos = get_current_position(cmd.id);
        //     double vel = get_current_velocity(cmd.id);

        //     feedback->ids.push_back(cmd.id);
        //     feedback->current_position.push_back(pos);
        //     feedback->current_velocity.push_back(vel);
        // }
        feedback->ids.push_back(RosCanConstants::VescIds::sampler_platform);
        feedback->current_position.push_back(get_current_position(RosCanConstants::VescIds::sampler_platform));
        feedback->current_velocity.push_back(get_current_velocity(RosCanConstants::VescIds::sampler_platform));
        feedback->current_effort.push_back(0.0);

        feedback->ids.push_back(RosCanConstants::VescIds::sampler_drill_mov);
        feedback->current_position.push_back(get_current_position(RosCanConstants::VescIds::sampler_drill_mov));
        feedback->current_velocity.push_back(get_current_velocity(RosCanConstants::VescIds::sampler_drill_mov));
        feedback->current_effort.push_back(0.0);

        feedback->ids.push_back(RosCanConstants::VescIds::sampler_drill);
        feedback->current_position.push_back(get_current_position(RosCanConstants::VescIds::sampler_drill));
        feedback->current_velocity.push_back(get_current_velocity(RosCanConstants::VescIds::sampler_drill));
        feedback->current_effort.push_back(0.0);


        goal_handle->publish_feedback(feedback);
        
        bool all_reached = true;
        for (const auto & cmd : goal->commands) {
          std::string joint_name;
          //RCLCPP_INFO(this->get_logger(), "Loop id check: %d", cmd.id);
          if (cmd.id == RosCanConstants::VescIds::sampler_platform) {
            joint_name = "platform_joint";
            // RCLCPP_INFO(this->get_logger(), "Checking id 1");
          }
          else if (cmd.id == RosCanConstants::VescIds::sampler_drill_mov) joint_name = "drill_joint";
          else if (cmd.id == RosCanConstants::VescIds::sampler_drill) joint_name = "rotor_joint";

          double pos = get_current_position(cmd.id);
          // RCLCPP_INFO(this->get_logger(), "Position of current slider:  %f", pos);
          // optional: double vel = current_velocity_[joint_name];
          double result_ = final_pos - pos;
          // RCLCPP_INFO(this->get_logger(), "Result of calculations:  %f", result_);
          if (cmd.id != RosCanConstants::VescIds::sampler_drill && std::fabs(result_) > tolerance) {
              all_reached = false;
              break;  // one joint not reached, keep looping
          }
      }
        // Check if target reached
        if (all_reached) {
            trajectory_msgs::msg::JointTrajectory hold;

            hold.joint_names = {joint_name};

            trajectory_msgs::msg::JointTrajectoryPoint p;
            p.positions = {get_current_position(current_id)};
            p.time_from_start = rclcpp::Duration::from_seconds(0.01);

            hold.points.push_back(p);


            if (current_slider_ == 1) {
                platform_pub_->publish(traj);
            } else if (current_slider_ == 2) {
                drill_pub_->publish(traj);
            }

            auto rotor_msg = std_msgs::msg::Float64MultiArray();
            rotor_msg.data = {0.0};
            rotor_velocity_pub_->publish(rotor_msg);
            result->success = true;
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "Goal succeeded! Reached");
            return;
        
        }
        
        rate.sleep();
    
    }
  }
 // class MoveLinearActionServer

}  // namespace action_tutorials_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(ros_deep_sampler::MoveLinearActionServer)