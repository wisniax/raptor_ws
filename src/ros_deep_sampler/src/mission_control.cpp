#include "ros_deep_sampler/mission_control.hpp"



namespace ros_deep_sampler{

    MissionControl::MissionControl(const rclcpp::NodeOptions & options)
  : Node("Mission_control_node", options)
  {

    missionTimer_ = this->create_timer(std::chrono::milliseconds(10), std::bind(&MissionControl::MissionUpdate, this));
 
    joints_ = std::make_unique<JointMovement>(this);
    

    mRoverStatus_ = this->create_subscription<rex_interfaces::msg::RoverStatus>(
    "/MQTT/RoverStatus",
    10,
    std::bind(&MissionControl::HandleRoverStatus, this, std::placeholders::_1)); 
    
    ////Change in the future
    PubFeedback_ = this->create_publisher<MissionMsg>("/MQTT/SamplerFeedback", 100);
    appFeedbackTimer_ = this->create_wall_timer(
     std::chrono::milliseconds(500), std::bind(&MissionControl::publishAppFeedback, this));

    // MeasurementFeedback_ = this->create_subscription<rex_interfaces::msg::SamplerFeedback>(
    //   RosCanConstants::RosTopics::can_sampler_status, 10,
    //   std::bind(&MissionControl::HandleMeasurementFeedback, this, std::placeholders::_1));

    mMissionCmd_ = this->create_subscription<MissionCmd>("/MQTT/MissionCommand", 10,
                    std::bind(&MissionControl::HandleMissionCmd, this, std::placeholders::_1));
    
    missionCmdMsg = std::make_shared<MissionCmd>();
    missionFeedbackMsg = std::make_shared<MissionMsg>();
    missionCmdMsg->mission_cmd = MissionCmd::STOP;
    missionFeedbackMsg->control_type = MissionMsg::NO_SAMPLER;
    missionFeedbackMsg->autonomy_state = MissionMsg::STATE_IDLE;

    // sampler_can_cmd_pub_ = get_node()->create_publisher<SamplerCanCmd>(
    //         RosCanConstants::RosTopics::can_sampler_cmd,10);
    
    // rex_interfaces::msg::RoverStatus init_msg;
    // init_msg.communication_state = RoverStatusMsg::COMMUNICATION_STATE_CLOSED;
    // init_msg.pad_connected = false;
    // init_msg.control_mode = RoverStatusMsg::CONTROL_MODE_ESTOP;
    // LastStatusMsg = std::make_shared<const RoverStatusMsg>(init_msg);
    // LastCtrlMsg = std::make_shared<const SamplerControlMsg>();

    RCLCPP_INFO(this->get_logger(), "Constructor executed");

    // mPubFeedback.platform_position = this->platform_position;
    // mPubFeedback.drill_position = this->drill_position;
    // mPubFeedback.drill_current = this->drill_velocity; 
  }

  void MissionControl::MissionUpdate()
{

    getFeedback(missionFeedbackMsg, *joints_);
    // Highest priority: invalid control mode
    if (ctrlType_ == ControlType::NO_SAMPLER)
    {
        stopExecuting();
        // mission_state_ = MissionState::STOPPED;
        return;
    }
    
    // Global mission commands
    switch (missionCmdMsg->mission_cmd)
    {
        case MissionCmd::ABORT:
            //abortMission();
            abortExecuting();
            return;

        case MissionCmd::STOP:
            //stopMission();
            stopExecuting();
            return;

        case MissionCmd::RESTART:
            //restartMission();
            restartAutonomy();
            missionCmdMsg->mission_cmd = MissionCmd::START;
            return;

        case MissionCmd::CALIBRATE:
            //startCalibration();
            calibrateSampler();
            if(autonomy_.getCalibrationState()){
               missionCmdMsg->mission_cmd = MissionCmd::STOP;
               autonomy_.resetStateVariables();
               autonomy_.setState(AutonomyController::AutonomyStates::IDLE);
            }
            return;

        case MissionCmd::START:
            // handled below
            mission_in_stop = false;
            // if (autonomy_stopped){
            //     autonomy_stopped = false;
            //     // autonomy_.setState(state_to_stop_);
            // }
            break;
    }

    // Control mode
    if (ctrlType_ == ControlType::MANUAL)
    {
        executeManual();
        autonomy_.resetStateVariables();
        autonomy_.setState(AutonomyController::AutonomyStates::IDLE);
        return;
    }

    if (ctrlType_ == ControlType::AUTONOMY)
    {
        new_mission_cmd = false;
        executeAutonomy();
        return;
    }
}

void MissionControl::setControlType(const RoverStatusMsg::ConstSharedPtr &msg){
    if (msg->communication_state == RoverStatusMsg::COMMUNICATION_STATE_OPENED &&
		   msg->control_mode == (CONTROL_MODE_DEEP_SAMPLER_AUTONOMY | CONTROL_MODE_SURFACE_SAMPLER_AUTONOMY)){
        ctrlType_ = ControlType::AUTONOMY; 
        return;
        }
    if (msg->communication_state == RoverStatusMsg::COMMUNICATION_STATE_OPENED &&
		   msg->control_mode == (CONTROL_MODE_DEEP_SAMPLER | CONTROL_MODE_SURFACE_SAMPLER)){
        ctrlType_ = ControlType::MANUAL; 
        return;
        }
    ctrlType_ = ControlType::NO_SAMPLER;
    return;
}

void MissionControl::HandleMissionCmd(const MissionCmd& missionCmd)
{
    missionCmdMsg->mission_cmd = missionCmd.mission_cmd;

    if (missionCmdMsg->mission_cmd == MissionCmd::CALIBRATE){
        return;
    } 

    if(ctrlType_ == ControlType::MANUAL){
        if (missionCmd.platform_movement != NO_CMD)
        {
            missionCmdMsg->platform_movement = missionCmd.platform_movement;
            platform_cmd_pending_ = true;
        }

        if (missionCmd.drill_movement != NO_CMD)
        {
            missionCmdMsg->drill_movement = missionCmd.drill_movement;
            drill_cmd_pending_ = true;
        }

        if (missionCmd.container_degrees != NO_CMD)
        {
            missionCmdMsg->container_degrees = missionCmd.container_degrees;
            container_cmd_pending_ = true;
        }

        if (missionCmd.drill_action != NO_CMD)
        {
            missionCmdMsg->drill_action = missionCmd.drill_action;
            drill_rotor_cmd_pending_ = true;
        }

        if (missionCmd.vacuum_suction != NO_CMD)
        {
            missionCmdMsg->vacuum_suction = missionCmd.vacuum_suction;
            vacuum_cmd_pending_ = true;
        }

        if (missionCmd.brush_rotation != NO_CMD)
        {
            missionCmdMsg->brush_rotation = missionCmd.brush_rotation;
            brush_cmd_pending_ = true;
        }

        // if (missionCmd.open_vacuum)
        // {
        //     missionCmdMsg->open_vacuum = missionCmd.open_vacuum;
        //     clamp_cmd_pending_ = true;
        // }
        missionCmdMsg->open_vacuum = missionCmd.open_vacuum;

        missionCmdMsg->mission_cmd = MissionCmd::START;


        retranslateSamplerCtrlMsg(missionCmdMsg);
    }

    //new_mission_cmd = true;
}



uint8_t MissionControl::getMissionCmd(){
  return missionCmdMsg->mission_cmd;
}

void MissionControl::HandleRoverStatus(const RoverStatusMsg::ConstSharedPtr &roverStatusMsg)
{
    setControlType(roverStatusMsg);
	// LastStatusMsg = roverStatusMsg;

}

void MissionControl::executeAutonomy(){
   
   if(autonomy_.getState() == AutonomyController::AutonomyStates::ABORT){
    missionCmdMsg->mission_cmd == MissionCmd::ABORT;
    stopExecuting();
    return;
   }
   autonomy_.executeAutonomy(
        sampler_state_,
        *missionCmdMsg,
        *missionFeedbackMsg,
        *joints_,
        this->get_logger());
}



void MissionControl::stopExecuting(){
     if(!mission_in_stop){
        std::string state_string = to_string();
        RCLCPP_INFO(this->get_logger(), "Mission STOP on state: %s", state_string.c_str());
        joints_->send_rotor_velocity(JointMovement::JointsIds::DRILL_ROTOR, 0.0);
        joints_->send_rotor_velocity(JointMovement::JointsIds::VACUUM_ROTOR, 0.0);
        joints_->send_rotor_velocity(JointMovement::JointsIds::BRUSH_ROTOR, 0.0);
        //move_client_->cancel_goal();
        if(ctrlType_ == ControlType::AUTONOMY){
            autonomy_stopped = true;
            autonomy_.resetStateVariables();
            // state_to_stop_ = autonomy_.getState();
        }
        
        // if (goal_sent){
        joints_->cancelMovement();
        //     goal_sent = false;
        // }
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

void MissionControl::restartAutonomy(){
    autonomy_.resetStateVariables();
    autonomy_.setState(AutonomyController::AutonomyStates::IDLE);
    return;
}

void MissionControl::calibrateSampler(){
    autonomy_.requestCalibration(*joints_, this->get_logger());
    
}

void MissionControl::executeManual(){
    autonomy_.stop(*joints_);
    //retranslateSamplerCtrlMsg(missionCmdMsg);
    return;
}

void MissionControl::abortExecuting(){
    stopExecuting();
    autonomy_.setState(AutonomyController::AutonomyStates::DONE);
    return;
}





  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//   void MissionControl::HandleMeasurementFeedback(const MeasurementMsg::ConstSharedPtr &measurementMsg){
//   weight_a = measurementMsg->weight_a;
//   ph = measurementMsg->ph;
// }

// bool MissionControl::get_measurements(){
//   if (weight_a != 0.0 && ph != 0.0){
//     return true;
//   }else{return false;}
// }

double MissionControl::getPlatformPosition(){
  return (double)sampler_state_.platform_position;
}


void MissionControl::retranslateSamplerCtrlMsg(
    const MissionCmd::SharedPtr& missionCmd)
{
    if (ctrlType_ != ControlType::MANUAL)
        return;

    // if (!new_mission_cmd)
    //     return;

    //new_mission_cmd = false;

    if (missionCmd->velocity_ctrl){
        double platform;
        double drill;
        double container;
        joints_->setJointsVel(missionCmd->platform_movement,  missionCmd->drill_movement,
                             missionCmd->container_degrees); // degrees as the velocity 

        if(platform_cmd_pending_){
            platform = missionCmd->platform_movement;
            platform_cmd_pending_ = false;
        }else{platform = 0.0;}
        if(drill_cmd_pending_){
            drill = missionCmd->drill_movement;
            drill_cmd_pending_ = false;
        }else{drill = 0.0;}
        if(container_cmd_pending_){
            container = missionCmd->drill_movement;
            container_cmd_pending_ = false;
        }else{container = 0.0;}
        
        joints_->setJointsVel(platform, drill, container);
    }else{
        joints_->setJointsVel(0.0, 0.0, 0.0);
        std::vector<JointMovement::JointCommand> commands;

        if(platform_cmd_pending_){
        commands.push_back({
            JointMovement::JointsIds::PLATFORM,
            joints_->get_current_position(
                JointMovement::JointsIds::PLATFORM),
            missionCmd->platform_movement,
            0.2});
            platform_cmd_pending_ = false;
        }else{
            commands.push_back({
                JointMovement::JointsIds::PLATFORM,
                joints_->get_current_position(
                    JointMovement::JointsIds::PLATFORM),
                joints_->get_current_position(
                    JointMovement::JointsIds::PLATFORM),
                0.2});
        }
    

        if(drill_cmd_pending_){
            commands.push_back({
                JointMovement::JointsIds::DRILL,
                joints_->get_current_position(
                    JointMovement::JointsIds::DRILL),
                missionCmd->drill_movement,
                0.2});
            drill_cmd_pending_ = false;
            }else{
                commands.push_back({
                    JointMovement::JointsIds::DRILL,
                    joints_->get_current_position(
                        JointMovement::JointsIds::DRILL),
                    joints_->get_current_position(
                        JointMovement::JointsIds::DRILL),
                    0.2});
            }

    if(container_cmd_pending_){
            commands.push_back({
            JointMovement::JointsIds::CONTAINER,
            joints_->get_current_position(
                JointMovement::JointsIds::CONTAINER),
            missionCmd->container_degrees,
            0.2});
        container_cmd_pending_ = false;
        }else{
            commands.push_back({
                JointMovement::JointsIds::CONTAINER,
                joints_->get_current_position(
                    JointMovement::JointsIds::CONTAINER),
                joints_->get_current_position(
                    JointMovement::JointsIds::CONTAINER),
                0.2});
        }

        joints_->moveJoints(commands);
    }

    // Rotors
    if(drill_rotor_cmd_pending_){
        joints_->send_rotor_velocity(
            JointMovement::JointsIds::DRILL_ROTOR,
            missionCmd->drill_action);
        drill_rotor_cmd_pending_ = false;
    }else{
        joints_->send_rotor_velocity(
            JointMovement::JointsIds::DRILL_ROTOR,
            joints_->get_current_velocity(JointMovement::JointsIds::DRILL_ROTOR));
    }

    if(vacuum_cmd_pending_){
        joints_->send_rotor_velocity(
            JointMovement::JointsIds::VACUUM_ROTOR,
            missionCmd->vacuum_suction);
        vacuum_cmd_pending_ = false;
    }else{
        joints_->send_rotor_velocity(
            JointMovement::JointsIds::VACUUM_ROTOR,
            joints_->get_current_velocity(JointMovement::JointsIds::VACUUM_ROTOR));
    }

    if(brush_cmd_pending_){
        joints_->send_rotor_velocity(
            JointMovement::JointsIds::BRUSH_ROTOR,
            missionCmd->brush_rotation);
        brush_cmd_pending_ = false;
    }else{
        joints_->send_rotor_velocity(
            JointMovement::JointsIds::BRUSH_ROTOR,
            joints_->get_current_velocity(JointMovement::JointsIds::BRUSH_ROTOR));
    }

    if(missionCmd->open_vacuum){
        joints_->open_clamp();
    }else{
       joints_->close_clamp();
    }
    
}


// bool MissionControl::drillStuck(){

//     if (std::abs(joints_->get_current_velocity(JointMovement::JointsIds::DRILL_ROTOR)) < MIN_SPEED)
//     {
//         if (!stall_timer_running_)
//         {
//             stall_start_time_ = this->now();
//             stall_timer_running_ = true;
//         }

//         if ((this->now() - stall_start_time_).seconds() > MIN_TIME)
//             return true;
//     }
//     else
//     {
//         stall_timer_running_ = false;
//     }

//     return false;
//}
void MissionControl::getFeedback(MissionMsg::SharedPtr &missionFeedback, JointMovement& joints_){
  missionFeedback->control_type = ctrl_to_Feedback();
  missionFeedback->autonomy_state = to_Feedback();
  missionFeedback->platform_pos = joints_.get_current_position(JointMovement::JointsIds::PLATFORM);
  missionFeedback->drill_pos = joints_.get_current_position(JointMovement::JointsIds::DRILL);
  missionFeedback->drill_rot_vel = joints_.get_current_velocity(JointMovement::JointsIds::DRILL_ROTOR);
  missionFeedback->brush_rot_vel = joints_.get_current_velocity(JointMovement::JointsIds::BRUSH_ROTOR);
  missionFeedback->vacuum_suction_vel = joints_.get_current_velocity(JointMovement::JointsIds::VACUUM_ROTOR);
  missionFeedback->container_pos = joints_.get_current_velocity(JointMovement::JointsIds::CONTAINER);
  joints_.JointStateFeedback(missionFeedback);
}


void MissionControl::publishAppFeedback(){
  // RCLCPP_INFO(this->get_logger(), "Current mission command is =  %d", missionCmdMsg->mission_cmd);
  // RCLCPP_INFO(this->get_logger(), "Current Rover status is =  %d", LastStatusMsg);
  std::string str_ = to_string();
   RCLCPP_INFO(
        this->get_logger(),
        "\n"
        "========== Mission Feedback ==========\n"
        "Control type:          %d\n"
        "Autonomy state:        %d\n"
        "Platform position:     %.3f\n"
        "Drill position:        %.3f\n"
        "Drill rotor velocity:  %.3f\n"
        "Brush rotor velocity:  %.3f\n"
        "Vacuum rotor velocity: %.3f\n"
        "Container position:    %.3f\n"
        "Goal state:            %d\n"
        "======================================",
        missionFeedbackMsg->control_type,
        missionFeedbackMsg->autonomy_state,
        missionFeedbackMsg->platform_pos,
        missionFeedbackMsg->drill_pos,
        missionFeedbackMsg->drill_rot_vel,
        missionFeedbackMsg->brush_rot_vel,
        missionFeedbackMsg->vacuum_suction_vel,
        missionFeedbackMsg->container_pos,
        missionFeedbackMsg->goal_state
    );

  PubFeedback_->publish(*missionFeedbackMsg);

}

uint8_t MissionControl::to_Feedback(){
    switch (autonomy_.getState())
    {
        case AutonomyController::AutonomyStates::IDLE:
            return MissionMsg::STATE_IDLE;

        // case AutonomyController::AutonomyStates::CALIBRATE_PLATFORM:
        //     return MissionMsg::STATE_CALIBRATE_PLATFORM;

        // case AutonomyController::AutonomyStates::CALIBRATE_DRILL:
        //     return MissionMsg::STATE_CALIBRATE_DRILL;

        // case AutonomyController::AutonomyStates::CALIBRATE_CONTAINER:
        //     return MissionMsg::STATE_CALIBRATE_CONTAINER;
        case AutonomyController::AutonomyStates::CALIBRATE_JOINTS:
            return MissionMsg::STATE_CALIBRATE_JOINTS;

        case AutonomyController::AutonomyStates::MOVE_PLATFORM_DOWN:
            return MissionMsg::STATE_MOVE_PLATFORM_DOWN;

        case AutonomyController::AutonomyStates::GET_SURFACE_SAMPLE:
            return MissionMsg::STATE_GET_SURFACE_SAMPLE;

        case AutonomyController::AutonomyStates::DRILLING:
            return MissionMsg::STATE_DRILLING;

        case AutonomyController::AutonomyStates::RECOVER_DRILL:
            return MissionMsg::STATE_RECOVER_DRILL;

        case AutonomyController::AutonomyStates::MOVE_DRILL_UP:
            return MissionMsg::STATE_MOVE_DRILL_UP;

        case AutonomyController::AutonomyStates::MOVE_PLATFORM_UP:
            return MissionMsg::STATE_MOVE_PLATFORM_UP;

        case AutonomyController::AutonomyStates::MOVE_CONTAINER:
            return MissionMsg::STATE_MOVE_CONTAINER;

        case AutonomyController::AutonomyStates::MOVE_DRILL_CLOSER:
            return MissionMsg::STATE_MOVE_DRILL_CLOSER;

        case AutonomyController::AutonomyStates::PUT_DEEP_SAMPLE:
            return MissionMsg::STATE_PUT_DEEP_SAMPLE;

        case AutonomyController::AutonomyStates::HIDE_DRILL:
            return MissionMsg::STATE_HIDE_DRILL;

        case AutonomyController::AutonomyStates::MOVE_PLATFORM_CLOSER:
            return MissionMsg::STATE_MOVE_PLATFORM_CLOSER;

        case AutonomyController::AutonomyStates::PUT_SURFACE_SAMPLE:
            return MissionMsg::STATE_PUT_SURFACE_SAMPLE;

        case AutonomyController::AutonomyStates::MOVE_PLATFORM_BACK:
            return MissionMsg::STATE_MOVE_PLATFORM_BACK;

        case AutonomyController::AutonomyStates::HIDE_CONTAINER:
            return MissionMsg::STATE_HIDE_CONTAINER;

        case AutonomyController::AutonomyStates::MEASURE_SAMPLES:
            return MissionMsg::STATE_MEASURE_SAMPLES;

        // case AutonomyController::AutonomyStates::STOP:
        //     return MissionMsg::STATE_STOP;

        // case AutonomyController::AutonomyStates::ABORT:
        //     return MissionMsg::STATE_ABORT;

        case AutonomyController::AutonomyStates::DONE:
            return MissionMsg::STATE_DONE;

        default:
            return MissionMsg::STATE_ABORT;  // or define a STATE_UNKNOWN if preferred
    }
}

uint8_t MissionControl::ctrl_to_Feedback(){
    switch(ctrlType_){
        case ControlType::NO_SAMPLER:
            return MissionMsg::NO_SAMPLER;
        break;

        case ControlType::AUTONOMY:
            return MissionMsg::AUTONOMY;
        break;

        case ControlType::MANUAL:
            return MissionMsg::MANUAL;
        break;

    }
}

std::string MissionControl::to_string()
{
    switch(autonomy_.getState()) {
        case AutonomyController::AutonomyStates::IDLE:
            return "IDLE";

        // case AutonomyController::AutonomyStates::CALIBRATE_PLATFORM:
        //     return "CALIBRATING PLATFORM (SETTING ORIGIN)";

        // case AutonomyController::AutonomyStates::CALIBRATE_DRILL:
        //     return "CALIBRATING DRILL (SETTING ORIGIN)";

        // case AutonomyController::AutonomyStates::CALIBRATE_CONTAINER:
        //     return "CALIBRATING CONTAINER (SETTING ORIGIN)";
        case AutonomyController::AutonomyStates::CALIBRATE_JOINTS:
            return "CALIBRATING ALL TRAJECTORY JOINTS";


        case AutonomyController::AutonomyStates::MOVE_PLATFORM_DOWN:
            return "MOVING PLATFORM DOWN";

        case AutonomyController::AutonomyStates::GET_SURFACE_SAMPLE:
            return "GETTING SURFACE SAMPLE";

        case AutonomyController::AutonomyStates::DRILLING:
            return "DRILLING";

        case AutonomyController::AutonomyStates::RECOVER_DRILL:
            return "RECOVERING DRILL";


        case AutonomyController::AutonomyStates::MOVE_DRILL_UP:
            return "MOVING DRILL UP";

        case AutonomyController::AutonomyStates::MOVE_PLATFORM_UP:
            return "MOVING PLATFORM UP";


        case AutonomyController::AutonomyStates::MOVE_CONTAINER:
            return "MOVING CONTAINER";

        case AutonomyController::AutonomyStates::MOVE_DRILL_CLOSER:
            return "MOVING DRILL CLOSER";

        case AutonomyController::AutonomyStates::PUT_DEEP_SAMPLE:
            return "PUTTING DEEP SAMPLE";

        case AutonomyController::AutonomyStates::HIDE_DRILL:
            return "HIDING DRILL";


        case AutonomyController::AutonomyStates::MOVE_PLATFORM_CLOSER:
            return "MOVING PLATFORM CLOSER";

        case AutonomyController::AutonomyStates::PUT_SURFACE_SAMPLE:
            return "PUTTING SURFACE SAMPLE";

        case AutonomyController::AutonomyStates::MOVE_PLATFORM_BACK:
            return "MOVING PLATFORM BACK";

        case AutonomyController::AutonomyStates::HIDE_CONTAINER:
            return "HIDING CONTAINER";


        case AutonomyController::AutonomyStates::MEASURE_SAMPLES:
            return "MEASURING SAMPLES";

        // case AutonomyController::AutonomyStates::MANUAL_CONTROL:
        //     return "MANUAL CONTROL";


        // case AutonomyController::AutonomyStates::STOP:
        //     return "STOPPED";

        // case AutonomyController::AutonomyStates::ABORT:
        //     return "ABORTED";

        case AutonomyController::AutonomyStates::DONE:
            return "DONE";

        default:
            return "UNKNOWN";
    }
}





} //name space 




