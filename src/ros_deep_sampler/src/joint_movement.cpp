#include "ros_deep_sampler/joint_movement.hpp"


namespace ros_deep_sampler{

JointMovement::JointMovement(rclcpp::Node *node)
: node_(node)
{
    position_client_ =
    rclcpp_action::create_client<FollowJointTrajectory>(
        node_,
        "/position_controller/follow_joint_trajectory");
    // platform_client_ =
    //     rclcpp_action::create_client<FollowJointTrajectory>(
    //         node_,
    //         "/platform_controller/follow_joint_trajectory");

    // drill_client_ =
    //     rclcpp_action::create_client<FollowJointTrajectory>(
    //         node_,
    //         "/drill_position_controller/follow_joint_trajectory");

    // container_client_ =
    //     rclcpp_action::create_client<FollowJointTrajectory>(
    //         node_,
    //         "/container_controller/follow_joint_trajectory");

    rotor_velocity_pub_ =
        node_->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/rotor_velocity_controller/commands", 10);
    vacuum_rotor_velocity_pub_ =
        node_->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/vacuum_rotor_velocity_controller/commands", 10);
    brush_rotor_velocity_pub_ =
        node_->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/brush_rotor_velocity_controller/commands", 10);

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

double JointMovement::get_current_position(JointsIds id){
  if(id == JointsIds::PLATFORM){
    return current_position_["platform_joint"];
  }
  if(id == JointsIds::DRILL){
    return current_position_["drill_joint"];
  }
  if(id == JointsIds::CONTAINER){
    return current_position_["container_joint"];
  }

}
double JointMovement::get_current_velocity(JointsIds id){
  if(id == JointsIds::PLATFORM){
    return current_velocity_["platform_joint"];
  }
  if(id == JointsIds::DRILL){
    return current_velocity_["drill_joint"];
  }
  if(id == JointsIds::DRILL_ROTOR){
    return current_velocity_["rotor_joint"];
  }
  if(id == JointsIds::CONTAINER){
    return current_velocity_["container_joint"];
  }
  if(id == JointsIds::VACUUM_ROTOR){
    return current_velocity_["vacuum_rotor_joint"];
  }
  if(id == JointsIds::BRUSH_ROTOR){
    return current_velocity_["brush_rotor_joint"];
  }
  

}

void JointMovement::send_rotor_velocity(JointsIds rotor_id, double vel){
      std_msgs::msg::Float64MultiArray msg;
      msg.data = {vel};
      if(rotor_id == JointsIds::DRILL_ROTOR){
        rotor_velocity_pub_->publish(msg);
      }
      if(rotor_id == JointsIds::VACUUM_ROTOR){
        vacuum_rotor_velocity_pub_->publish(msg);
      }
      if(rotor_id == JointsIds::BRUSH_ROTOR){
        brush_rotor_velocity_pub_->publish(msg);
      }
      
  }

JointMovement::MotionProfile JointMovement::createProfile(
    const JointCommand& cmd,
    double accel_time){

    MotionProfile p;

    p.start = cmd.start_position;
    p.goal = cmd.final_position;

    double distance =
        cmd.final_position - cmd.start_position;

    p.direction =
        (distance >= 0.0) ? 1.0 : -1.0;

    distance = std::abs(distance);

    p.max_velocity = cmd.max_velocity;

    p.accel_time = accel_time;

    p.accel =
        p.max_velocity / p.accel_time;

    p.accel_dist =
        0.5 *
        p.accel *
        p.accel_time *
        p.accel_time;

    p.cruise_dist =
        distance - 2.0 * p.accel_dist;

    if (p.cruise_dist < 0.0)
    {
        // Triangle profile

        p.accel_time =
            std::sqrt(distance / p.accel);

        p.accel_dist =
            distance / 2.0;

        p.cruise_dist = 0.0;
        p.cruise_time = 0.0;

        p.max_velocity =
            p.accel * p.accel_time;
    }
    else
    {
        p.cruise_time =
            p.cruise_dist /
            p.max_velocity;
    }

    p.total_time =
        2.0 * p.accel_time +
        p.cruise_time;

    return p;
}
void JointMovement::evaluateProfile(
    const MotionProfile& p,
    double t,
    double& pos,
    double& vel)
{
    if (t >= p.total_time)
    {
        pos = p.goal;
        vel = 0.0;
        return;
    }

    double local_pos;

    if (t < p.accel_time)
    {
        vel = p.accel * t;

        local_pos =
            0.5 *
            p.accel *
            t *
            t;
    }
    else if (t < p.accel_time + p.cruise_time)
    {
        double tc =
            t - p.accel_time;

        vel =
            p.max_velocity;

        local_pos =
            p.accel_dist +
            p.max_velocity * tc;
    }
    else
    {
        double td =
            t -
            p.accel_time -
            p.cruise_time;

        vel =
            p.max_velocity -
            p.accel * td;

        local_pos =
            p.accel_dist +
            p.cruise_dist +
            p.max_velocity * td -
            0.5 * p.accel * td * td;
    }

    pos =
        p.start +
        p.direction * local_pos;

    vel =
        p.direction * vel;
}

void JointMovement::generateMultiJointsTrajectory(trajectory_msgs::msg::JointTrajectory &traj,
             const std::vector<JointCommand>& commands,
             double accel_time,
             double dt){
    
    traj.joint_names.clear();
    traj.points.clear();

    std::vector<MotionProfile> profiles;

    //-------------------------------------------------
    // Create profiles
    //-------------------------------------------------

    for (const auto& cmd : commands)
    {
        traj.joint_names.push_back(
            getJointName(cmd.id));

        profiles.push_back(
            createProfile(
                cmd,
                accel_time));
    }

    //-------------------------------------------------
    // Find longest motion
    //-------------------------------------------------

    double total_time = 0.0;

    for (const auto& p : profiles)
    {
        total_time =
            std::max(
                total_time,
                p.total_time);
    }

    //-------------------------------------------------
    // Generate trajectory points
    //-------------------------------------------------

    for (double t = 0.0;
         t <= total_time + 1e-6;
         t += dt)
    {
        trajectory_msgs::msg::JointTrajectoryPoint point;

        for (const auto& p : profiles)
        {
            double pos;
            double vel;

            evaluateProfile(
                p,
                t,
                pos,
                vel);

            point.positions.push_back(pos);
            point.velocities.push_back(vel);
        }

        point.time_from_start =
            rclcpp::Duration::from_seconds(t);

        traj.points.push_back(point);
    }

    //-------------------------------------------------
    // Exact endpoint
    //-------------------------------------------------

    if (!traj.points.empty())
    {
        auto& last = traj.points.back();

        for (size_t i = 0;
             i < commands.size();
             i++)
        {
            last.positions[i] =
                commands[i].final_position;

            last.velocities[i] = 0.0;
        }
    }            

    
 }

 void JointMovement::moveJoints(const std::vector<JointCommand>& commands){
    trajectory_msgs::msg::JointTrajectory traj;

    generateMultiJointsTrajectory(
        traj,
        commands,
        0.5,
        0.02);

    sendTrajectory(traj);
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
    double time_to_position =  std::fabs(final_pos)/vel;
    trajectory_msgs::msg::JointTrajectory traj;
    trajectory_msgs::msg::JointTrajectoryPoint p;
    p.positions = {final_pos, final_pos, -final_pos}; 
    RCLCPP_INFO(node_->get_logger(), "Final position: %f",final_pos);  
    RCLCPP_INFO(node_->get_logger(), "Time to position: %f",time_to_position);
    p.time_from_start = rclcpp::Duration::from_seconds(time_to_position);
    traj.joint_names = {"platform_joint",
    "drill_joint",
    "container_joint"};
    traj.points.push_back(p);

    sendTrajectory(traj);

    return;

}

std::string JointMovement::getJointName(JointsIds id)
{
    switch(id)
    {
        case JointsIds::PLATFORM:
            return "platform_joint";

        case JointsIds::DRILL:
            return "drill_joint";

        case JointsIds::CONTAINER:
            return "container_joint";

        default:
            return "";
    }
}

// void JointMovement::moveJoint(const std::vector<JointCommand>& commands){
//     trajectory_msgs::msg::JointTrajectory traj;
//     for (const auto &cmd : commands)
//     {
//     //    traj.joint_names.push_back(getJointName(cmd.id));
        
//         generateTrajectory(
//             traj,
//             getJointName(cmd.id),
//             get_current_position(cmd.id),
//             cmd.position,
//             cmd.max_velocity,
//             cmd.calibration);

//     }
//     sendTrajectory(traj);
// }

// void JointMovement::calibrateDrill(double vel){
//     float final_pos;
    
//     std::string joint_name;
//     joint_name = "drill_joint";
//     final_pos = 0.6; //move up on max 

//     trajectory_msgs::msg::JointTrajectory traj;
//     trajectory_msgs::msg::JointTrajectoryPoint p;
//     p.positions = {final_pos}; //dist or final_pos?
//     double time_to_position =  std::fabs(final_pos)/vel;
//     RCLCPP_INFO(node_->get_logger(), "Final position: %f",final_pos);  
//     RCLCPP_INFO(node_->get_logger(), "Time to position: %f",time_to_position);
//     p.time_from_start = rclcpp::Duration::from_seconds(time_to_position);
//     traj.joint_names.push_back(joint_name);
//     traj.points.push_back(p);

//     sendTrajectory(traj, Jo);

//     return;

// }

void JointMovement::setTrajectoryStatus(bool status){
    trajectory_finished_ = status;
}

void JointMovement::sendTrajectory(
    trajectory_msgs::msg::JointTrajectory &traj)
{
    if(!position_client_->wait_for_action_server(
        std::chrono::seconds(2)))
    {
        RCLCPP_ERROR(
            node_->get_logger(),
            "JTC unavailable");

        goal_state = MissionMsg::GOAL_FAILED;
        return;
    }


    FollowJointTrajectory::Goal goal;
    goal.trajectory = traj;


    auto options =
        rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();


    trajectory_finished_ = false;
    goal_state = MissionMsg::GOAL_SENT;


    options.goal_response_callback =
        [this](TJCGoalHandle::SharedPtr handle)
        {
            if(!handle)
            {
                RCLCPP_ERROR(
                    node_->get_logger(),
                    "Trajectory rejected");

                goal_state = MissionMsg::GOAL_FAILED;
                return;
            }

            RCLCPP_INFO(
                node_->get_logger(),
                "Trajectory accepted");

            goal_state = MissionMsg::GOAL_ACCEPTED;

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

                    goal_state = MissionMsg::GOAL_SUCCEEDED;
                    break;


                case rclcpp_action::ResultCode::CANCELED:

                    RCLCPP_INFO(
                        node_->get_logger(),
                        "Trajectory canceled");

                    goal_state = MissionMsg::GOAL_CANCELED;
                    break;


                default:

                    RCLCPP_ERROR(
                        node_->get_logger(),
                        "Trajectory failed");

                    goal_state = MissionMsg::GOAL_FAILED;
                    break;
            }
        };


    position_client_->async_send_goal(goal, options);
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
        position_client_->async_cancel_goal(
            active_tjc_goal_);

        RCLCPP_INFO(
            node_->get_logger(),
            "Trajectory cancellation requested");
    }
}

void JointMovement::JointStateFeedback(MissionMsg::SharedPtr &feedbackMsg){
    if(prev_goal_state == MissionMsg::GOAL_ACCEPTED){
        goal_state = MissionMsg::GOAL_EXECUTING;
    }
    feedbackMsg->goal_state = goal_state;
    prev_goal_state = goal_state;

}

}
