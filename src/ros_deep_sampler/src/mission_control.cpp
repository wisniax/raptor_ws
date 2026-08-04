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
    LastMissionCmdMsg = missionCmdMsg;
    missionFeedbackMsg = std::make_shared<MissionMsg>();
    missionCmdMsg->mission_cmd = MissionCmd::STOP;
    missionFeedbackMsg->mission_state = MissionMsg::STATE_IDLE;
    
    rex_interfaces::msg::RoverStatus init_msg;
    init_msg.communication_state = RoverStatusMsg::COMMUNICATION_STATE_CLOSED;
    init_msg.pad_connected = false;
    init_msg.control_mode = RoverStatusMsg::CONTROL_MODE_ESTOP;
    LastStatusMsg = std::make_shared<const RoverStatusMsg>(init_msg);
    LastCtrlMsg = std::make_shared<const SamplerControlMsg>();

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

uint8_t MissionControl::getMissionCmd(){
  return missionCmdMsg->mission_cmd;
}

void MissionControl::retranslateSamplerCtrlMsg(const MissionCmd &missionCmd){

    JointMovement::JointCommand cmd;
    cmd.id = JointMovement::JointsIds::PLATFORM;
    cmd.position = missionCmd->platform_movement;
    cmd.max_velocity = 0.1;
    commands.push_back(cmd);
    joints_->moveJoints(commands);
    commands = {
        {JointsIds::PLATFORM,  joints_->get_current_position(JointsIds::PLATFORM),  missionCmd->platform_movement,  0.2},
        {JointsIds::DRILL,     joints_->get_current_position(JointsIds::DRILL),     missionCmd->drill_movement,    0.2},
        {JointsIds::CONTAINER, joints_->get_current_position(JointsIds::CONTAINER), missionCmd->container_degrees_a, 0.2}
    };
}


bool MissionControl::checkCommands(uint8_t current_mission_cmd)
{

    if(current_mission_cmd == MissionCmd::STOP){
          stopExecuting();
          return false;
        }
    mission_in_stop = false;
    if(current_mission_cmd == MissionCmd::ABORT){
        state_ = State::ABORT;
        return false;
    }
    if(ctrlType_ == CONTROL_TYPE::MANUAL && current_mission_cmd == MissionCmd::START){
        state_ = State::MANUAL_CONTROL;
        return false;
    }

    if(ctrlType_ == CONTROL_TYPE::AUTONOMY && current_mission_cmd == MissionCmd::START){
        return true;
    }
   return false;

}
/////

void MissionControl::setControlType(const RoverStatusMsg::ConstSharedPtr &msg){
    if (msg->communication_state == RoverStatusMsg::COMMUNICATION_STATE_OPENED &&
		   msg->control_mode == (CONTROL_MODE_DEEP_SAMPLER_AUTONOMY | CONTROL_MODE_SURFACE_SAMPLER_AUTONOMY)){
        ctrlType_ = CONTROL_TYPE::AUTONOMY; 
        return;
        }
    if (msg->communication_state == RoverStatusMsg::COMMUNICATION_STATE_OPENED &&
		   msg->control_mode == (CONTROL_MODE_DEEP_SAMPLER | CONTROL_MODE_SURFACE_SAMPLER)){
        ctrlType_ = CONTROL_TYPE::MANUAL; 
        return;
        }
    ctrlType_ = CONTROL_TYPE::NO_SAMPLER;
    return;
}

// bool MissionControl::isSamplerMode(const RoverStatusMsg::ConstSharedPtr &msg)
// {

// 	return msg->communication_state == RoverStatusMsg::COMMUNICATION_STATE_OPENED &&
// 		   msg->control_mode == (CONTROL_MODE_DEEP_SAMPLER_AUTONOMY | CONTROL_MODE_SURFACE_SAMPLER_AUTONOMY);
// }

void MissionControl::HandleSamplerCtl(const SamplerControlMsg::ConstSharedPtr &samplerCtlMsg)
{
	if (ctrlType_ == CONTROL_TYPE::MANUAL)
		LastCtrlMsg = samplerCtlMsg;

}

void MissionControl::HandleRoverStatus(const RoverStatusMsg::ConstSharedPtr &roverStatusMsg)
{
    setControlType(roverStatusMsg);
	// LastStatusMsg = roverStatusMsg;

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

    if (std::abs(joints_->get_current_velocity(JointMovement::JointsIds::DRILL_ROTOR)) < MIN_SPEED)
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


void MissionControl::stopExecuting(){
     if(!mission_in_stop){
        std::string state_string = to_string(state_);
        RCLCPP_INFO(this->get_logger(), "Mission STOP on state: %s", state_string.c_str());
        joints_->send_rotor_velocity(JointMovement::JointsIds::DRILL_ROTOR, 0.0);
        //move_client_->cancel_goal();
        if (goal_sent){
            joints_->cancelMovement();
            goal_sent = false;
        }
        // while(joints_->isGoalCanceled() != 1){}
        mission_in_stop = true;
        RCLCPP_INFO(this->get_logger(), "Mission Stopped successfully");
        }
    // if(checkCommands(current_mission_cmd) && current_mission_cmd == MissionCmd::START){
    // std::string state_string = to_string(state_);
    // RCLCPP_INFO(this->get_logger(), "Continue the mission from state: %s", state_string.c_str());
    // mission_in_stop = false;
    // }
}


void MissionControl::getFeedback(MissionMsg::SharedPtr &feedbackMsg){
  feedbackMsg->platform_pos = joints_->get_current_position(JointMovement::JointsIds::PLATFORM);
  feedbackMsg->drill_pos = joints_->get_current_position(JointMovement::JointsIds::DRILL);
  feedbackMsg->drill_rot_vel = joints_->get_current_velocity(JointMovement::JointsIds::DRILL_ROTOR);
  feedbackMsg->container_a_pos = joints_->get_current_velocity(JointMovement::JointsIds::CONTAINER);
  joints_->JointStateFeedback(feedbackMsg);
}

void MissionControl::AppFeedbackPublish(){
  // RCLCPP_INFO(this->get_logger(), "Current mission command is =  %d", missionCmdMsg->mission_cmd);
  // RCLCPP_INFO(this->get_logger(), "Current Rover status is =  %d", LastStatusMsg);
  std::string str_ = to_string(state_);
  RCLCPP_INFO(this->get_logger(), "Current state is =  %s", str_.c_str());
  RCLCPP_INFO(this->get_logger(), "Platform pos =  %f", missionFeedbackMsg->platform_pos);
  RCLCPP_INFO(this->get_logger(), "Drill pos =  %f", missionFeedbackMsg->drill_pos);
  RCLCPP_INFO(this->get_logger(), "Drill vel =  %f", missionFeedbackMsg->drill_rot_vel);
  RCLCPP_INFO(this->get_logger(), "Current goal state =  %d", missionFeedbackMsg->goal_state);
 
 
  
  // rex_interfaces::msg::SamplerFeedback msg;
  // msg.platform_pos = platform_position;
  // msg.drill_pos = drill_position;
  // msg.drill_rot_vel = drill_velocity;

  // PubFeedback_->publish(msg);

}

uint8_t MissionControl::to_Feedback(State s){
    switch (s)
    {
        case State::IDLE:
            return MissionMsg::STATE_IDLE;

        case State::CALIBRATE_PLATFORM:
            return MissionMsg::STATE_CALIBRATE_PLATFORM;

        case State::CALIBRATE_DRILL:
            return MissionMsg::STATE_CALIBRATE_DRILL;

        case State::CALIBRATE_CONTAINER:
            return MissionMsg::STATE_CALIBRATE_CONTAINER;

        case State::MOVE_PLATFORM_DOWN:
            return MissionMsg::STATE_MOVE_PLATFORM_DOWN;

        case State::GET_SURFACE_SAMPLE:
            return MissionMsg::STATE_GET_SURFACE_SAMPLE;

        case State::DRILLING:
            return MissionMsg::STATE_DRILLING;

        case State::RECOVER_DRILL:
            return MissionMsg::STATE_RECOVER_DRILL;

        case State::MOVE_DRILL_UP:
            return MissionMsg::STATE_MOVE_DRILL_UP;

        case State::MOVE_PLATFORM_UP:
            return MissionMsg::STATE_MOVE_PLATFORM_UP;

        case State::MOVE_CONTAINER:
            return MissionMsg::STATE_MOVE_CONTAINER;

        case State::MOVE_DRILL_CLOSER:
            return MissionMsg::STATE_MOVE_DRILL_CLOSER;

        case State::PUT_DEEP_SAMPLE:
            return MissionMsg::STATE_PUT_DEEP_SAMPLE;

        case State::HIDE_DRILL:
            return MissionMsg::STATE_HIDE_DRILL;

        case State::MOVE_PLATFORM_CLOSER:
            return MissionMsg::STATE_MOVE_PLATFORM_CLOSER;

        case State::PUT_SURFACE_SAMPLE:
            return MissionMsg::STATE_PUT_SURFACE_SAMPLE;

        case State::MOVE_PLATFORM_BACK:
            return MissionMsg::STATE_MOVE_PLATFORM_BACK;

        case State::HIDE_CONTAINER:
            return MissionMsg::STATE_HIDE_CONTAINER;

        case State::MEASURE_SAMPLES:
            return MissionMsg::STATE_MEASURE_SAMPLES;

        case State::STOP:
            return MissionMsg::STATE_STOP;

        case State::ABORT:
            return MissionMsg::STATE_ABORT;

        case State::DONE:
            return MissionMsg::STATE_DONE;

        default:
            return MissionMsg::STATE_ABORT;  // or define a STATE_UNKNOWN if preferred
    }
}

std::string MissionControl::to_string(State s)
{
    switch(s) {
        case State::IDLE:
            return "IDLE";

        case State::CALIBRATE_PLATFORM:
            return "CALIBRATING PLATFORM (SETTING ORIGIN)";

        case State::CALIBRATE_DRILL:
            return "CALIBRATING DRILL (SETTING ORIGIN)";

        case State::CALIBRATE_CONTAINER:
            return "CALIBRATING CONTAINER (SETTING ORIGIN)";


        case State::MOVE_PLATFORM_DOWN:
            return "MOVING PLATFORM DOWN";

        case State::GET_SURFACE_SAMPLE:
            return "GETTING SURFACE SAMPLE";

        case State::DRILLING:
            return "DRILLING";

        case State::RECOVER_DRILL:
            return "RECOVERING DRILL";


        case State::MOVE_DRILL_UP:
            return "MOVING DRILL UP";

        case State::MOVE_PLATFORM_UP:
            return "MOVING PLATFORM UP";


        case State::MOVE_CONTAINER:
            return "MOVING CONTAINER";

        case State::MOVE_DRILL_CLOSER:
            return "MOVING DRILL CLOSER";

        case State::PUT_DEEP_SAMPLE:
            return "PUTTING DEEP SAMPLE";

        case State::HIDE_DRILL:
            return "HIDING DRILL";


        case State::MOVE_PLATFORM_CLOSER:
            return "MOVING PLATFORM CLOSER";

        case State::PUT_SURFACE_SAMPLE:
            return "PUTTING SURFACE SAMPLE";

        case State::MOVE_PLATFORM_BACK:
            return "MOVING PLATFORM BACK";

        case State::HIDE_CONTAINER:
            return "HIDING CONTAINER";


        case State::MEASURE_SAMPLES:
            return "MEASURING SAMPLES";


        case State::STOP:
            return "STOPPED";

        case State::ABORT:
            return "ABORTED";

        case State::DONE:
            return "DONE";

        default:
            return "UNKNOWN";
    }
}





} //name space 




