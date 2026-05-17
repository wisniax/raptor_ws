#include "ros_deep_sampler/mission_control.hpp"



namespace ros_deep_sampler{

    MissionControl::MissionControl(const rclcpp::NodeOptions & options)
  : Node("Mission_control_node", options)
  {

    missionTimer_ = this->create_timer(std::chrono::milliseconds(10), std::bind(&MissionControl::statesLoop, this));
    move_client_ = std::make_shared<MoveLinearActionClient>(
    rclcpp::NodeOptions());
    
    
    sub_ = this->create_subscription<std_msgs::msg::String>(
    "/command",
    10,
    std::bind(&MissionControl::MissionCheck, this, std::placeholders::_1));   

    mStatus_ = this->create_subscription<rex_interfaces::msg::RoverStatus>(
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

    //mPubCanCtrl_ = this->create_publisher<sampler_motion_interfaces::msg::SamplerCanEx>("/SamplerCanCom", 100);

    
    rex_interfaces::msg::RoverStatus init_msg;
    init_msg.communication_state = RoverStatusMsg::COMMUNICATION_STATE_CLOSED;
    init_msg.pad_connected = false;
    init_msg.control_mode = RoverStatusMsg::CONTROL_MODE_ESTOP;
    LastStatusMsg = std::make_shared<const RoverStatusMsg>(init_msg);

    RCLCPP_INFO(this->get_logger(), "Constructor executed");


    // mPubFeedback.platform_position = this->platform_position;
    // mPubFeedback.drill_position = this->drill_position;
    // mPubFeedback.drill_current = this->drill_velocity; 
  }

int MissionControl::getPlatformPosition(){
  return (int)platform_position;
}

/////

bool MissionControl::isSamplerMode(const RoverStatusMsg::ConstSharedPtr &msg)
{
  // RCLCPP_INFO(this->get_logger(), "Com state mode of msg: %d", msg->communication_state);
  // RCLCPP_INFO(this->get_logger(), "Control mode of msg: %d", msg->control_mode);
  // RCLCPP_INFO(this->get_logger(), "Com state Opened: %d", RoverStatusMsg::COMMUNICATION_STATE_OPENED);
  // RCLCPP_INFO(this->get_logger(), "Control mode sampler: %d", RoverStatusMsg::CONTROL_MODE_SAMPLER);

	return msg->communication_state == RoverStatusMsg::COMMUNICATION_STATE_OPENED &&
		   msg->control_mode == RoverStatusMsg::CONTROL_MODE_SAMPLER;
}

void MissionControl::HandleSamplerCtl(const SamplerControlMsg::ConstSharedPtr &samplerCtlMsg)
{
	if (isSamplerMode(LastStatusMsg))
		LastCtrlMsg = samplerCtlMsg;

 //sampler_motion_interfaces::msg::SamplerCanEx msg;
 //msg.platform_pos = LastCtrlMsg->platform_movement

  
	// else
	// 	RCLCPP_WARN_THROTTLE(mNh->get_logger(), *mNh->get_clock(), 5 * 60 * 1000, // Throttle duration (5 minutes)
	// 						 "When non-sampler mode is selected, incoming SamplerControl MQTT messages are discarded.");
}

void MissionControl::HandleRoverStatus(const RoverStatusMsg::ConstSharedPtr &roverStatusMsg)
{
  // RCLCPP_INFO(this->get_logger(), "Recieved status with mode: %f", roverStatusMsg->control_mode);
	// bool stop_sampler = false;
	// if (!isSamplerMode(roverStatusMsg) && isSamplerMode(LastStatusMsg))
	// 	stop_sampler = true;

	// if (isSamplerMode(roverStatusMsg) && !isSamplerMode(LastStatusMsg))
	// 	stop_sampler = true;
  // RCLCPP_INFO(this->get_logger(), "Recieved Rover status msg");
  // RCLCPP_INFO(this->get_logger(), "Com state mode: %d", roverStatusMsg->communication_state);
  // RCLCPP_INFO(this->get_logger(), "COntrol mode: %d", roverStatusMsg->control_mode);
  // RCLCPP_WARN(this->get_logger(), "Callback ptr: %p", roverStatusMsg.get());
  // RCLCPP_WARN(this->get_logger(), "Stored ptr: %p", LastStatusMsg.get());
	LastStatusMsg = roverStatusMsg;

	// if (stop_sampler)
	// 	stopSampler();
}


//////
void MissionControl::statesLoop(){
    // RCLCPP_INFO(this->get_logger(), "statesLoop running");
    switch(state_){
      case State::IDLE:
        //RCLCPP_WARN(this->get_logger(), "Read ptr: %p", LastStatusMsg.get());
        // RCLCPP_INFO(this->get_logger(), "Is sampler mode: %d", MissionControl::isSamplerMode(LastStatusMsg));
        if(MissionControl::isSamplerMode(LastStatusMsg)){
          RCLCPP_INFO(this->get_logger(), "Starting deep  sampling mission");
          goal_sent = false;
          if (getPlatformPosition() != 0){
            RCLCPP_INFO(this->get_logger(), "Starting callibration");
            state_ = State::MOVE_UP_CALIBRATION;
          }
        else{
          RCLCPP_INFO(this->get_logger(), "Starting moving down");
          state_ = State::MOVE_PLATFORM_DOWN;
        }
        }

      // waiting for mission_start callback
      break;

      case State::MOVE_UP_CALIBRATION:
     
        if(!(MissionControl::isSamplerMode(LastStatusMsg))){
          RCLCPP_INFO(this->get_logger(), "Mission is canceled");
          state_to_abort = state_;
          state_ = State::ABORT;
          break;
        }
        if(!goal_sent){
          cmd1.id = RosCanConstants::VescIds::sampler_platform;
          cmd1.velocity = 0.05;
          cmd1.position = 0.2;
          commands.push_back(cmd1);
          move_client_->send_goal(commands);
          RCLCPP_INFO(this->get_logger(), "Starting moving up");
          move_client_->send_goal(commands);
          goal_sent = true;
        }
        
        if(move_client_->get_goal_status() == 1){
          RCLCPP_INFO(this->get_logger(), "Finishing moving up");
          goal_sent = false;
          move_client_->set_goal_status(goal_sent);
          state_ = State::MOVE_PLATFORM_DOWN; 
        }
        
        break;

      case State::MOVE_PLATFORM_DOWN:
        if(!(MissionControl::isSamplerMode(LastStatusMsg))){
          RCLCPP_INFO(this->get_logger(), "Mission is canceled");
          state_to_abort = state_;
          state_ = State::ABORT;
          break;
        }
        if(!goal_sent){
          RCLCPP_INFO(this->get_logger(), "Starting moving down");
          cmd1.id = RosCanConstants::VescIds::sampler_platform;
          cmd1.velocity = 0.04;
          cmd1.position = -0.4;
          commands.push_back(cmd1);
          // cmd2.id = 2;
          // cmd2.velocity = 0.0;
          // cmd2.position = 0.0;
          // commands.push_back(cmd2);
          // cmd3.id = 3;
          // cmd3.velocity = 0.0;
          // cmd3.position = 0.0;
          // commands.push_back(cmd3);
          move_client_->send_goal(commands);
          goal_sent = true;
        }
        
        if(move_client_->get_goal_status() == 1){
          time_between_states++;
          if(time_between_states > 1){
            RCLCPP_INFO(this->get_logger(), "Finishing moving down");
            time_between_states = 0;
            goal_sent = false;
            move_client_->set_goal_status(goal_sent);
            commands.clear();
            state_ = State::DRILLING; 
          }
         
          
        }
        break;

      case State::DRILLING:
        if(!(MissionControl::isSamplerMode(LastStatusMsg))){
          RCLCPP_INFO(this->get_logger(), "Mission is canceled");
          state_to_abort = state_;
          state_ = State::ABORT;
          break;
        }
        if(!goal_sent){
          RCLCPP_INFO(this->get_logger(), "Starting drilling");
          // cmd1.id = 1;
          // cmd1.velocity = 0.04;
          // cmd1.position = -0.1;
          // commands.push_back(cmd1);
          cmd2.id = RosCanConstants::VescIds::sampler_drill_mov;
          cmd2.velocity = 0.04;
          cmd2.position = -0.35;
          commands.push_back(cmd2);
          cmd3.id = RosCanConstants::VescIds::sampler_drill;
          cmd3.velocity = 4.0;
          cmd3.position = 0.0;
          commands.push_back(cmd3);
          move_client_->send_goal(commands);
          goal_sent = true;
        }
        
        if(move_client_->get_goal_status() ==1){
          time_between_states++;
          if(time_between_states > 1){
            RCLCPP_INFO(this->get_logger(), "Finishing moving down during drillling");
            time_between_states = 0;
            goal_sent = false;
            move_client_->set_goal_status(goal_sent);
            commands.clear();
            state_ = State::MOVE_DRILL_UP; 
          }
          
         
        }
        break;

        case State::MOVE_DRILL_UP:
          if(!(MissionControl::isSamplerMode(LastStatusMsg))){
            RCLCPP_INFO(this->get_logger(), "Mission is canceled");
            state_to_abort = state_;
            state_ = State::ABORT;
            break;
          }
          if(!goal_sent){
            RCLCPP_INFO(this->get_logger(), "Moving drill up");
            // cmd1.id = 1;
            // cmd1.velocity = 0.0;
            // cmd1.position = 0.0
            // commands.push_back(cmd1);
            cmd2.id = RosCanConstants::VescIds::sampler_drill_mov;
            cmd2.velocity = 0.04;
            cmd2.position = 0.0;
            commands.push_back(cmd2);
            // cmd3.id = 3;
            // cmd3.velocity = 0.0;
            // cmd3.position = 0.0;
            // commands.push_back(cmd3);
            move_client_->send_goal(commands);
            goal_sent = true;
          }
          
          if(move_client_->get_goal_status() == 1){
            time_between_states++;
            if(time_between_states > 100){
                RCLCPP_INFO(this->get_logger(), "Finish moving drill up");
                time_between_states =0;
                goal_sent = false;
                move_client_->set_goal_status(goal_sent);
                commands.clear();
                state_ = State::MOVE_PLATFORM_UP; 
              }
          }   
        break; 
        
        case State::MOVE_PLATFORM_UP:
          if(!(MissionControl::isSamplerMode(LastStatusMsg))){
              RCLCPP_INFO(this->get_logger(), "Mission is canceled");
              state_to_abort = state_;
              state_ = State::ABORT;
              break;
            }
            if(!goal_sent){
              RCLCPP_INFO(this->get_logger(), "Moving Platform up");
              cmd1.id = RosCanConstants::VescIds::sampler_platform;
              cmd1.velocity = 0.04;
              cmd1.position = 0.0;
              commands.push_back(cmd1);
              // cmd2.id = 2;
              // cmd2.velocity = 0.04;
              // cmd2.position = 0.1;
              // commands.push_back(cmd2);
              // cmd3.id = 3;
              // cmd3.velocity = 0.0;
              // cmd3.position = 0.0;
              // commands.push_back(cmd3);
              move_client_->send_goal(commands);
              goal_sent = true;
            }
            
            if(move_client_->get_goal_status() ==1){
              time_between_states++;
              if(time_between_states > 100){
                  RCLCPP_INFO(this->get_logger(), "Finish moving drill up");
                  time_between_states =0;
                  goal_sent = false;
                  move_client_->set_goal_status(goal_sent);
                  commands.clear();
                  state_ = State::IDLE; 
                }
            }   
        break; 

      
        case State::DONE:
        break;

        case State::ABORT:
            std::string state_string = to_string(state_to_abort);
            RCLCPP_INFO(this->get_logger(), "Mission Abortion on state: %s", state_string.c_str());
            move_client_->cancel_goal();
            while(move_client_->get_goal_status() != 2){}
            RCLCPP_INFO(this->get_logger(), "Mission Aborted successfully");
            state_= State::DONE;

        break;
    }
}


void MissionControl::MissionCheck(std_msgs::msg::String::SharedPtr msg){
  RCLCPP_INFO(this->get_logger(), "Received: %s", msg->data.c_str());
  mission_commands= msg->data;

    // if (msg->data == "mission_start"){
     
    //     if(state_ == State::IDLE){
    //         state_ = State::MOVING;
    //         RCLCPP_INFO(this->get_logger(), "Current state: %s", to_string(state_).c_str());            
    //         //move_client_->send_goal();
    //     }

    //}
  
  
  //RCLCPP_INFO(this->get_logger(), "Current state: %s", to_string(state_).c_str());
}


void MissionControl::AppFeedbackPublish(){
  platform_position = move_client_->get_position(RosCanConstants::VescIds::sampler_platform);
  drill_position = move_client_->get_position(RosCanConstants::VescIds::sampler_drill_mov);

  drill_velocity = move_client_->get_velocity(RosCanConstants::VescIds::sampler_drill);

  rex_interfaces::msg::SamplerFeedback msg;
  msg.platform_pos = platform_position;
  msg.drill_pos = drill_position;
  msg.drill_rot_vel = drill_velocity;

  PubFeedback_->publish(msg);

}

std::shared_ptr<MoveLinearActionClient> MissionControl::get_move_client() {
    return move_client_;
}



} //name space 




