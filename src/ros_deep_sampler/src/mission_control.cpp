#include "ros_deep_sampler/mission_control.hpp"



namespace ros_deep_sampler{

    MissionControl::MissionControl(const rclcpp::NodeOptions & options)
  : Node("Mission_control_node", options)
  {

    missionTimer_ = this->create_timer(std::chrono::milliseconds(10), std::bind(&MissionControl::statesLoop, this));
 
    joints_ = std::make_unique<JointMovement>(this);
    

    mRoverStatus_ = this->create_subscription<rex_interfaces::msg::RoverStatus>(
    "/MQTT/RoverStatus",
    10,
    std::bind(&MissionControl::HandleRoverStatus, this, std::placeholders::_1)); 
    
    mSamplerCtrl_ = this->create_subscription<rex_interfaces::msg::SamplerControl>(
      "/MQTT/SamplerControl",
      10,
      std::bind(&MissionControl::HandleSamplerCtl, this, std::placeholders::_1));

    // SubStatus = this->create_subscription<rex_interfaces::msg::SamplerControl>(
    //     "/MQTT/SamplerControl",
    //     rclcpp::QoS(10), std::bind(&MissionControl::MissionCheck, this, std::placeholders::_1));

    PubFeedback_ = this->create_publisher<rex_interfaces::msg::SamplerFeedback>("/MQTT/SamplerFeedback", 100);
    AppFeedbackTimer_ = this->create_wall_timer(
     std::chrono::milliseconds(500), std::bind(&MissionControl::AppFeedbackPublish, this));

    MeasurementFeedback_ = this->create_subscription<rex_interfaces::msg::SamplerFeedback>(
      RosCanConstants::RosTopics::can_sampler_status, 10,
      std::bind(&MissionControl::HandleMeasurementFeedback, this, std::placeholders::_1));
    
    rotor_velocity_pub_ = this->create_publisher<std_msgs::msg::Float64>(
              "/rotor_joint_cmd", 10);

    mMissionCmd_ = this->create_subscription<MissionCmd>("/MQTT/MissionCommand", 10,
                    std::bind(&MissionControl::HandleMissionCmd, this, std::placeholders::_1));
    
    missionCmdMsg = std::make_shared<MissionCmd>();
    missionFeedbackMsg = std::make_shared<MissionMsg>();
    missionCmdMsg->mission_cmd = MissionCmd::IDLE;
    missionFeedbackMsg->mission_state = MissionMsg::STATE_IDLE;
    
    rex_interfaces::msg::RoverStatus init_msg;
    init_msg.communication_state = RoverStatusMsg::COMMUNICATION_STATE_CLOSED;
    init_msg.pad_connected = false;
    init_msg.control_mode = RoverStatusMsg::CONTROL_MODE_ESTOP;
    LastStatusMsg = std::make_shared<const RoverStatusMsg>(init_msg);

    RCLCPP_INFO(this->get_logger(), "Constructor executed");

    stall_start_time_ = this->now();


    // mPubFeedback.platform_position = this->platform_position;
    // mPubFeedback.drill_position = this->drill_position;
    // mPubFeedback.drill_current = this->drill_velocity; 
  }

int MissionControl::getPlatformPosition(){
  return (int)platform_position;
}

void MissionControl::send_rotor_velocity(double vel){
      std_msgs::msg::Float64 msg;
      msg.data = {vel};
      rotor_velocity_pub_->publish(msg);
}

void MissionControl::HandleMissionCmd(const MissionCmd &missionCmd){
  missionCmdMsg->mission_cmd = missionCmd.mission_cmd;
}

uint8_t MissionControl::CheckMissionCmd(){
  return missionCmdMsg->mission_cmd;
}


bool MissionControl::checkFlags(uint8_t current_mission_cmd)
{
    if (current_mission_cmd == MissionCmd::ABORT)
    {
        state_to_abort = state_;
        state_ = State::ABORT;
        return false;
    }

    if (current_mission_cmd == MissionCmd::STOP)
    {
        state_to_stop = state_;
        state_ = State::STOP;
        return false;
    }

    if (!isSamplerMode(LastStatusMsg))
    {
        state_to_stop = state_;
        state_ = State::STOP;
        return false;
    }

    return true;
}
/////

bool MissionControl::isSamplerMode(const RoverStatusMsg::ConstSharedPtr &msg)
{

	return msg->communication_state == RoverStatusMsg::COMMUNICATION_STATE_OPENED &&
		   msg->control_mode == RoverStatusMsg::CONTROL_MODE_SAMPLER;
}

void MissionControl::HandleSamplerCtl(const SamplerControlMsg::ConstSharedPtr &samplerCtlMsg)
{
	if (isSamplerMode(LastStatusMsg))
		LastCtrlMsg = samplerCtlMsg;

}

void MissionControl::HandleRoverStatus(const RoverStatusMsg::ConstSharedPtr &roverStatusMsg)
{

	LastStatusMsg = roverStatusMsg;

}

void MissionControl::HandleMeasurementFeedback(const MeasurementMsg::ConstSharedPtr &measurementMsg){
  weight_a = measurementMsg->weight_a;
  ph = measurementMsg->ph;
}

bool MissionControl::get_measurements(){
  if (weight_a != 0.0 && ph != 0.0){
    return true;
  }else{return false;}
}

bool MissionControl::drillStuck(){

    if (std::abs(joints_->get_current_velocity(missionFeedbackMsg->ROTOR)) < MIN_SPEED)
    {
        if (!stall_timer_running_)
        {
            stall_start_time_ = this->now();
            stall_timer_running_ = true;
        }

        if ((this->now() - stall_start_time_).seconds() > MIN_TIME)
            return true;
    }
    else
    {
        stall_timer_running_ = false;
    }

    return false;
}


//////
void MissionControl::statesLoop(){
    // RCLCPP_INFO(this->get_logger(), "statesLoop running");
    uint8_t current_mission_cmd = CheckMissionCmd();
    missionFeedbackMsg->mission_state = to_Feedback(state_);
    getFeedback(missionFeedbackMsg);

    switch(state_){
      case State::IDLE:
        if(MissionControl::isSamplerMode(LastStatusMsg) && current_mission_cmd == MissionCmd::START){
          if(!calibrate_drill){
            state_ = State::CALIBRATE_DRILL;
          }else if(!calibrate_platform){
            state_ = State::CALIBRATE_PLATFORM;
          }

          if(calibrate_platform && calibrate_drill){
            state_ = State::MOVE_PLATFORM_DOWN;
            }

            
          }
      // waiting for mission_start callback
        break;

      case State::CALIBRATE_PLATFORM:
        if (!checkFlags(current_mission_cmd)){
          break;
        }
       
        if(!goal_sent){
          RCLCPP_INFO(this->get_logger(), "Starting PLATFORM CALIBRATION");
          joints_->calibratePlatform(0.02);
          goal_sent = true;
        }
        
        if(std::fabs(0.0 - joints_->get_current_position(1)) < 0.01){
          // time_between_states++;
          // if(time_between_states >100){
          RCLCPP_INFO(this->get_logger(), "Finishing moving up");
          goal_sent = false;
          calibrate_platform = true;
          joints_->cancelMovement();
       
          joints_->setTrajectoryStatus(false);
          time_between_states = 0;
          state_ = State::IDLE; 
          //}
         
        }
        
      break;

      case State::CALIBRATE_DRILL:
     
        if (!checkFlags(current_mission_cmd)){
          break;
        }

        if(!goal_sent){
          RCLCPP_INFO(this->get_logger(), "Starting DRILL CALIBRATION");
          joints_->calibrateDrill(0.02);
          goal_sent = true;
        }
        
        
        if(std::fabs(0.0 - joints_->get_current_position(2)) < 0.01){
          // time_between_states++;
          // if(time_between_states >100){
          RCLCPP_INFO(this->get_logger(), "Finishing moving up");
          goal_sent = false;
          calibrate_drill = true;
          joints_->cancelMovement();
          //while(joints_->isGoalCanceled() != 1){}
          joints_->setTrajectoryStatus(false);
          time_between_states = 0;
          state_ = State::IDLE; 
          //}
         
        }
        
      break;

      case State::MOVE_PLATFORM_DOWN:
         if (!checkFlags(current_mission_cmd)){
          break;
        }

        if(!goal_sent){
          //joints_->setGoalStatus(false);
          RCLCPP_INFO(this->get_logger(), "Starting moving down");
          JointMovement::JointCommand cmd;
          cmd.id = 1;
          cmd.position = -0.4;
          cmd.max_velocity = 0.2;
          commands.push_back(cmd);
          joints_->moveJoints(commands);
          goal_sent = true;
        }
        
        if(joints_->isTrajectoryFinished() == 1){
          time_between_states++;
          if(time_between_states > 100){
            RCLCPP_INFO(this->get_logger(), "Finishing moving down");
            time_between_states = 0;
            goal_sent = false;
            //move_client_->set_goal_status(goal_sent);
            commands.clear();
            state_ = State::DRILLING; 
          }
         
          
        }
        break;

      case State::DRILLING:
        if (!checkFlags(current_mission_cmd)){
          break;
        }

        if(!goal_sent){
          RCLCPP_INFO(this->get_logger(), "Starting drilling");
          JointMovement::JointCommand cmd;
          cmd.id = 2;
          cmd.position = -0.35;
          cmd.max_velocity = 0.2;
          commands.push_back(cmd);
          joints_->moveJoints(commands);
          joints_->send_rotor_velocity(15.0);
          goal_sent = true;
        }

        if (drillStuck()){
          RCLCPP_INFO(this->get_logger(), "Drill got stuck");
          joints_->send_rotor_velocity(0.0);
          joints_->cancelMovement();
          joints_->setTrajectoryStatus(false);
          time_between_states = 0;
          goal_sent = false;
          commands.clear();
          state_ = State::RECOVER_DRILL;
          break;
        }
        
        if(joints_->isTrajectoryFinished() ==1){
          time_between_states++;
          if(time_between_states > 100){
            RCLCPP_INFO(this->get_logger(), "Finishing moving down during drillling");
            time_between_states = 0;
            goal_sent = false;
            // move_client_->set_goal_status(goal_sent);
            commands.clear();
            joints_->send_rotor_velocity(0.0);
            recovery_attempt = 0;
            state_ = State::MOVE_DRILL_UP; 
          }
          
         
        }
        break;

        case State::MOVE_DRILL_UP:
           if (!checkFlags(current_mission_cmd)){
            break;
          }
          if(!goal_sent){
            RCLCPP_INFO(this->get_logger(), "Moving drill up");
            JointMovement::JointCommand cmd;
            cmd.id = 2;
            cmd.position = 0.01;
            cmd.max_velocity = 0.2;
            commands.push_back(cmd);
            joints_->moveJoints(commands);
            goal_sent = true;
          }
          
          if(joints_->isTrajectoryFinished() == 1){
            time_between_states++;
            if(time_between_states > 100){
                RCLCPP_INFO(this->get_logger(), "Finish moving drill up");
                time_between_states =0;
                goal_sent = false;
                // move_client_->set_goal_status(goal_sent);
                commands.clear();
                state_ = State::MOVE_PLATFORM_UP; 
              }
          }   
        break; 
        
        case State::MOVE_PLATFORM_UP:
          if (!checkFlags(current_mission_cmd)){
            break;
          }
            if(!goal_sent){
              RCLCPP_INFO(this->get_logger(), "Moving Platform up");
              JointMovement::JointCommand cmd;
              cmd.id = 1;
              cmd.position = 0.01;
              cmd.max_velocity = 0.2;
              commands.push_back(cmd);
              joints_->moveJoints(commands);
              goal_sent = true;
            }
            
            if(joints_->isTrajectoryFinished()  ==1){
              time_between_states++;
              if(time_between_states > 100){
                  RCLCPP_INFO(this->get_logger(), "Finish moving Platform up");
                  time_between_states =0;
                  goal_sent = false;
                  commands.clear();
                  joints_->setTrajectoryStatus(false);
                  state_ = State::MEASURE_SAMPLE; 
                }
            }   
        break; 

        case State::MEASURE_SAMPLE:
           if (!checkFlags(current_mission_cmd)){
            break;
          }
          switch(measurement_step){
            case MEASURE_STATE::MOVE_CONTAINER:
              if(!goal_sent){
                RCLCPP_INFO(this->get_logger(), "Starting moving container");
                JointMovement::JointCommand cmd;
                cmd.id = 3;
                cmd.position = -1.57;
                cmd.max_velocity = 0.6;
                commands.push_back(cmd);
                joints_->moveJoints(commands);
                measurement_step = MEASURE_STATE::MOVE_DRILL_CLOSER;
                goal_sent = true;
              }
              break;

            case MEASURE_STATE::MOVE_DRILL_CLOSER:
              if(!goal_sent){
                  RCLCPP_INFO(this->get_logger(), "Move drill closr to container");
                  JointMovement::JointCommand cmd;
                  cmd.id = 2;
                  cmd.position = -0.2;
                  cmd.max_velocity = 0.1;
                  commands.push_back(cmd);
                  joints_->moveJoints(commands);
                  measurement_step = MEASURE_STATE::PUT_SAMPLE;
                  // move_client_->send_goal(commands);
                  goal_sent = true;
                }
              break;

            case MEASURE_STATE::PUT_SAMPLE:
                if(rotation_time == 0){
                  joints_->send_rotor_velocity(5.0);
                }
              rotation_time ++;
              if(rotation_time > 300 && !goal_sent){
                joints_->send_rotor_velocity(0.0);
                JointMovement::JointCommand cmd;
                cmd.id = 2;
                cmd.position = 0.01;
                cmd.max_velocity = 0.2;
                commands.push_back(cmd);
                joints_->moveJoints(commands);
                measurement_step = MEASURE_STATE::MOVE_CONTAINER_BACK;
                rotation_time =0;
                goal_sent = true;
                // move_client_->set_goal_status(goal_sent);
              }

              break;
              
              
            case MEASURE_STATE::MOVE_CONTAINER_BACK:
              if(get_measurements()){
                if(!goal_sent){
                RCLCPP_INFO(this->get_logger(), "Starting moving container back");
                JointMovement::JointCommand cmd;
                cmd.id = 3;
                cmd.position = 0.01;
                cmd.max_velocity = 0.6;
                commands.push_back(cmd);
                joints_->moveJoints(commands);
                goal_sent = true;
                measurement_step = MEASURE_STATE::FINISH;
                }
              }
              break;
          }


          if(joints_->isTrajectoryFinished() ==1){
            time_between_states++;
            if(time_between_states > 100){
              //RCLCPP_INFO(this->get_logger(), "Measurement step is: %d", measurement_step);
              time_between_states = 0;
              goal_sent = false;
              joints_->setTrajectoryStatus(false);
              //move_client_->set_goal_status(goal_sent);
              commands.clear();
              if(measurement_step == MEASURE_STATE::FINISH){
                RCLCPP_INFO(this->get_logger(), "Finish measurements");
                state_ = State::DONE; 
              }
               
            }
          
          }
        break;

      
        case State::DONE:
        break;

        case State::RECOVER_DRILL:
          if (!checkFlags(current_mission_cmd)){
            break;
          }
          if (recovery_attempt > MAX_RECOVERY_ATTEMPT){
              state_to_abort = state_;
              state_ = State::ABORT;
            }

          switch(recover_state){
            case RECOVER_STATE::LIFT_UP:
            if(!goal_sent){
              RCLCPP_INFO(this->get_logger(), "Lift drill a bit up to recover");
              JointMovement::JointCommand cmd;
              cmd.id = missionFeedbackMsg->DRILL;
              cmd.position = joints_->get_current_position(cmd.id) + 0.05;
              cmd.max_velocity = 0.1;
              commands.push_back(cmd);
              joints_->moveJoints(commands);
              joints_->send_rotor_velocity(-5.0);
              goal_sent = true;
              stall_timer_running_ = false;
            }

            if(joints_->isTrajectoryFinished() ==1){
              time_between_states++;
              if(time_between_states > 50){
                RCLCPP_INFO(this->get_logger(), "Finishing moving up");
                time_between_states = 0;
                goal_sent = false;
                joints_->setTrajectoryStatus(false);
                commands.clear();
                recover_state = RECOVER_STATE::WAIT;
                break;
              }
            }
            break;

            case RECOVER_STATE::WAIT:
              rotation_time++;
              if(rotation_time>10){
                 joints_->send_rotor_velocity(12.0);
                 stall_timer_running_ = false;
                 rotation_time = 0;
                 recover_state = RECOVER_STATE::CHECK_ROTATION;
              }
            break;
            case RECOVER_STATE::CHECK_ROTATION:
              time_between_states++;
              if(time_between_states > 100){
                time_between_states =0;
                if (drillStuck()){
                  joints_->send_rotor_velocity(0.0);
                  goal_sent = false;
                  recover_state = RECOVER_STATE::LIFT_UP;
                  recovery_attempt++;
                }else{
                  joints_->send_rotor_velocity(0.0);
                  goal_sent = false;
                  recover_state = RECOVER_STATE::LIFT_UP;
                  recovery_attempt = 0;
                  state_ = State::DRILLING;
                  break;
                }
              }

            break;
          }
          

        break;

        case State::STOP:
          if(!mission_in_stop){
            std::string state_string = to_string(state_to_stop);
            RCLCPP_INFO(this->get_logger(), "Mission STOP on state: %s", state_string.c_str());
            joints_->send_rotor_velocity(0.0);
            //move_client_->cancel_goal();
            joints_->cancelMovement();
            // while(joints_->isGoalCanceled() != 1){}
            mission_in_stop = true;
            RCLCPP_INFO(this->get_logger(), "Mission Stopped successfully");
            state_= State::STOP;
          }
          if(MissionControl::isSamplerMode(LastStatusMsg) && current_mission_cmd == MissionCmd::START){
            state_ = state_to_stop;
            std::string state_string = to_string(state_to_stop);
            RCLCPP_INFO(this->get_logger(), "Continue the mission from state: %s", state_string.c_str());
            mission_in_stop = false;
          }
        
        break;

        case State::ABORT:
            joints_->send_rotor_velocity(0.0);
            std::string state_string = to_string(state_to_abort);
            RCLCPP_INFO(this->get_logger(), "Mission Abortion on state: %s", state_string.c_str());
            //move_client_->cancel_goal();
            joints_->cancelMovement();
            // while(joints_->isGoalCanceled() != 1){}
            RCLCPP_INFO(this->get_logger(), "Mission Aborted successfully");
            state_= State::DONE;

        break;
    }
}

void MissionControl::getFeedback(MissionMsg::SharedPtr &feedbackMsg){
  feedbackMsg->platform_pos = joints_->get_current_position(MissionMsg::PLATFORM);
  feedbackMsg->drill_pos = joints_->get_current_position(MissionMsg::DRILL);
  feedbackMsg->drill_rot_vel = joints_->get_current_velocity(MissionMsg::ROTOR);
  feedbackMsg->container_a_pos = joints_->get_current_velocity(MissionMsg::CONTAINER_A);
  joints_->JointStateFeedback(feedbackMsg);
}

void MissionControl::AppFeedbackPublish(){
  RCLCPP_INFO(this->get_logger(), "Current state is =  %d", missionFeedbackMsg->mission_state);
  RCLCPP_INFO(this->get_logger(), "Platform pos =  %f", missionFeedbackMsg->platform_pos);
  RCLCPP_INFO(this->get_logger(), "Drill pos =  %f", missionFeedbackMsg->drill_pos);
  RCLCPP_INFO(this->get_logger(), "Drill vel =  %f", missionFeedbackMsg->drill_rot_vel);
  RCLCPP_INFO(this->get_logger(), "Current Joint =  %d", missionFeedbackMsg->joint_id);
  RCLCPP_INFO(this->get_logger(), "Current goal state =  %d", missionFeedbackMsg->goal_state);
 
  
  // rex_interfaces::msg::SamplerFeedback msg;
  // msg.platform_pos = platform_position;
  // msg.drill_pos = drill_position;
  // msg.drill_rot_vel = drill_velocity;

  // PubFeedback_->publish(msg);

}





} //name space 




