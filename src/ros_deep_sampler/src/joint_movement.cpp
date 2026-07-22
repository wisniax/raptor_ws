#include "ros_deep_sampler/joint_movement.hpp"


namespace ros_deep_sampler{

JointMovement::JointMovement(rclcpp::Node *node)
: node_(node)
{
    platform_client_ =
        rclcpp_action::create_client<FollowJointTrajectory>(
            node_,
            "/platform_controller/follow_joint_trajectory");

    drill_client_ =
        rclcpp_action::create_client<FollowJointTrajectory>(
            node_,
            "/drill_position_controller/follow_joint_trajectory");

    container_client_ =
        rclcpp_action::create_client<FollowJointTrajectory>(
            node_,
            "/container_controller/follow_joint_trajectory");

    rotor_velocity_pub_ =
        node_->create_publisher<std_msgs::msg::Float64>(
            "/rotor_joint_cmd", 10);

    joint_sub_ =
        node_->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states",
            10,
            std::bind(
                &JointMovement::jointStateCallback,
                this,
                std::placeholders::_1));
}

void JointMovement::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
      // RCLCPP_INFO(this->get_logger(), "Received State Callback");
      for (size_t i = 0; i < msg->name.size(); ++i) {
        const std::string & name = msg->name[i];
        current_position_[name] = msg->position[i];
        current_velocity_[name] = msg->velocity[i];
    }
    
  }

double JointMovement::get_current_position(int id){
  if(id == 1){
    return current_position_["platform_joint"];
  }
  if(id == 2){
    return current_position_["drill_joint"];
  }
  if(id == 3){
    return current_position_["container_joint"];
  }

}
double JointMovement::get_current_velocity(int id){
  if(id == 1){
    return current_velocity_["platform_joint"];
  }
  if(id == 2){
    return current_velocity_["drill_joint"];
  }
  if(id == 4){
    return current_velocity_["rotor_joint"];
  }
  if(id == 3){
    return current_velocity_["container_joint"];
  }
  

}

void JointMovement::send_rotor_velocity(double vel){
      std_msgs::msg::Float64 msg;
      msg.data = {vel};
      rotor_velocity_pub_->publish(msg);
  }

  void JointMovement::generateTrajectory(
      trajectory_msgs::msg::JointTrajectory &traj,
      const std::string &joint_name,
      double start_pos,
      double final_pos,
      double max_velocity,
      bool is_calibration,
      double accel_time,
      double dt)
  {
     
    

      traj.joint_names.push_back(joint_name);

      double distance = final_pos - start_pos;
      double direction = (distance >= 0.0) ? 1.0 : -1.0;
      distance = std::abs(distance);

      // acceleration
      double accel = max_velocity / accel_time;

      // distance during acceleration
      double accel_dist = 0.5 * accel * accel_time * accel_time;

      double cruise_dist = distance - 2.0 * accel_dist;
      double cruise_time = 0.0;

      // Triangle profile if motion is too short
      if (cruise_dist < 0.0)
      {
          accel_time = std::sqrt(distance / accel);
          accel_dist = distance / 2.0;
          cruise_dist = 0.0;
          cruise_time = 0.0;
          max_velocity = accel * accel_time;
      }
      else
      {
          cruise_time = cruise_dist / max_velocity;
      }

      double total_time = 2.0 * accel_time + cruise_time;

      for (double t = 0.0; t <= total_time + 1e-6; t += dt)
      {
          double pos;
          double vel;

          if (t < accel_time)
          {
              // acceleration
              vel = accel * t;
              pos = 0.5 * accel * t * t;
          }
          else if (t < accel_time + cruise_time)
          {
              // cruise
              double tc = t - accel_time;
              vel = max_velocity;
              pos = accel_dist + max_velocity * tc;
          }
          else
          {
              // deceleration
              double td = t - accel_time - cruise_time;
              vel = max_velocity - accel * td;

              pos = accel_dist +
                    cruise_dist +
                    max_velocity * td -
                    0.5 * accel * td * td;
          }

          trajectory_msgs::msg::JointTrajectoryPoint point;

          point.positions.push_back(
              start_pos + direction * pos);

          point.velocities.push_back(
              direction * vel);

          point.time_from_start =
              rclcpp::Duration::from_seconds(t);

          traj.points.push_back(point);
      }

      // Ensure exact endpoint
      traj.points.back().positions[0] = final_pos;
      traj.points.back().velocities[0] = 0.0;
  }

void JointMovement::calibratePlatform(double vel){
    float final_pos;
    
    std::string joint_name;
    joint_name = "platform_joint";
    final_pos = 0.6; //move up on max 

    trajectory_msgs::msg::JointTrajectory traj;
    trajectory_msgs::msg::JointTrajectoryPoint p;
    p.positions = {final_pos}; //dist or final_pos?
    double time_to_position =  std::fabs(final_pos)/vel;
    RCLCPP_INFO(node_->get_logger(), "Final position: %f",final_pos);  
    RCLCPP_INFO(node_->get_logger(), "Time to position: %f",time_to_position);
    p.time_from_start = rclcpp::Duration::from_seconds(time_to_position);
    traj.joint_names.push_back(joint_name);
    traj.points.push_back(p);

    sendTrajectory(traj, 1);

    return;

}

std::string JointMovement::getJointName(int id)
{
    switch(id)
    {
        case 1:
            return "platform_joint";

        case 2:
            return "drill_joint";

        case 3:
            return "container_joint";

        default:
            return "";
    }
}

void JointMovement::moveJoints(const std::vector<JointCommand>& commands){
    for (const auto &cmd : commands)
    {
        trajectory_msgs::msg::JointTrajectory traj;

        generateTrajectory(
            traj,
            getJointName(cmd.id),
            get_current_position(cmd.id),
            cmd.position,
            cmd.max_velocity,
            cmd.calibration);

        sendTrajectory(traj, cmd.id);
    }
}

void JointMovement::calibrateDrill(double vel){
    float final_pos;
    
    std::string joint_name;
    joint_name = "drill_joint";
    final_pos = 0.6; //move up on max 

    trajectory_msgs::msg::JointTrajectory traj;
    trajectory_msgs::msg::JointTrajectoryPoint p;
    p.positions = {final_pos}; //dist or final_pos?
    double time_to_position =  std::fabs(final_pos)/vel;
    RCLCPP_INFO(node_->get_logger(), "Final position: %f",final_pos);  
    RCLCPP_INFO(node_->get_logger(), "Time to position: %f",time_to_position);
    p.time_from_start = rclcpp::Duration::from_seconds(time_to_position);
    traj.joint_names.push_back(joint_name);
    traj.points.push_back(p);

    sendTrajectory(traj, 2);

    return;

}

void JointMovement::setTrajectoryStatus(bool status){
    trajectory_finished_ = status;
}

void JointMovement::sendTrajectory(trajectory_msgs::msg::JointTrajectory &traj, int current_slider){
    
    if(current_slider == 1)
        active_client_ = platform_client_;
    else if(current_slider == 2)
        active_client_ = drill_client_;
    else if(current_slider == 3)
        active_client_ = container_client_;


    if(!active_client_->wait_for_action_server(
        std::chrono::seconds(2)))
    {
        RCLCPP_ERROR(
            node_->get_logger(),
            "JTC unavailable");
        return;
    }


    FollowJointTrajectory::Goal goal;
    goal.trajectory = traj;


    auto options =
        rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();

    trajectory_finished_ = false;
    options.goal_response_callback =
        [this](TJCGoalHandle::SharedPtr handle)
        {
            if(!handle)
            {
                RCLCPP_ERROR(
                    node_->get_logger(),
                    "Trajectory rejected");
                return;
            }

            RCLCPP_INFO(
                node_->get_logger(),
                "Trajectory accepted");

            active_tjc_goal_ = handle;
           
        };


    options.result_callback =
        [this](const TJCGoalHandle::WrappedResult &result)
        {
            trajectory_finished_ = true;
            active_tjc_goal_.reset();
            switch(result.code)
            {
                case rclcpp_action::ResultCode::SUCCEEDED:
                    RCLCPP_INFO(
                        node_->get_logger(),
                        "Trajectory finished");
                    
                    break;

                case rclcpp_action::ResultCode::CANCELED:
                    RCLCPP_INFO(
                        node_->get_logger(),
                        "Trajectory canceled");
                    break;

                default:
                    RCLCPP_ERROR(
                        node_->get_logger(),
                        "Trajectory failed");
                    break;
            }
        };


    active_client_->async_send_goal(goal, options);
    // result_future_ =
    //           active_client_->async_get_result(active_tjc_goal_);

            
}

bool JointMovement::isTrajectoryFinished()
{
    return trajectory_finished_;
    // if(!result_future_.valid())
    //     return false;


    // return result_future_.wait_for(
    //     std::chrono::seconds(0))
    //     == std::future_status::ready;
}

void JointMovement::cancelMovement()
{

    if(active_tjc_goal_)
    {
        active_client_->async_cancel_goal(
            active_tjc_goal_);

        RCLCPP_INFO(
            node_->get_logger(),
            "Trajectory cancelled");
    }

}


}
