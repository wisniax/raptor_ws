#include "ros_deep_sampler/autonomy_controller.hpp"


namespace ros_deep_sampler{




AutonomyController::AutonomyStates AutonomyController::getState()const{
    return state_;
};

void AutonomyController::setState(AutonomyStates state){
    state_ = state;
}

bool AutonomyController::getCalibrationState(){
    return calibrate_platform;
}

void AutonomyController::requestCalibration(JointMovement& joints_,
                                            const rclcpp::Logger& logger){
    if(!goal_sent){
        RCLCPP_INFO(logger, "Starting JOINTS CALIBRATION");
        joints_.calibrateJoints(0.02);
        goal_sent = true;
    }
    
    if(std::fabs(0.0 - joints_.get_current_position(JointMovement::JointsIds::PLATFORM)) < 0.01){
        // if(std::fabs(0.0 - joints_.get_current_position(JointMovement::JointsIds::PLATFORM)) < 0.01 &&             !!!!!!!!!!!!!!!!!
        //     std::fabs(0.0 - joints_.get_current_position(JointMovement::JointsIds::DRILL)) < 0.01 &&               !!!!!!!!!!!!!!!!!!
        //     std::fabs(0.0 - joints_.get_current_position(JointMovement::JointsIds::CONTAINER)) < 0.01){ do other}  !!!!!!!!!!!!!!!!!!!
        
        RCLCPP_INFO(logger, "Finishing moving up");
        goal_sent = false;
        calibrate_platform = true;
        joints_.cancelMovement();
    
        joints_.setTrajectoryStatus(false);
        return;
        //state_ = AutonomyStates::MOVE_PLATFORM_DOWN; 
        //}   
    }
    calibrate_platform = false;
    return;

}

void AutonomyController::resetStateVariables(){
    goal_sent = false;
    commands.clear();
    time_between_states = 0;
    rotation_time = 0;
    recovery_attempt = 0;
    calibrate_platform = false;
}

void AutonomyController::stop(JointMovement& joints_){
  
    if(goal_sent){
        joints_.stopMotors();
        //move_client_->cancel_goal();
        joints_.cancelMovement();
        resetStateVariables();
    }
    
}

void AutonomyController::checkStall(
        SamplerState& sampler,
        const rclcpp::Time& now,
        const rclcpp::Logger& logger){

    // Is the drill actually rotating?
    bool drill_not_moving =
        std::fabs(sampler.rotor_velocity) < MIN_SPEED;

    if (sampler.rotor_in_action && drill_not_moving)
    {
        // Start the stall timer
        if (!sampler.drill_stall_active_)
        {
            sampler.drill_stall_active_ = true;
            sampler.stall_start_time_ = now;

            RCLCPP_WARN(
                logger,
                "Possible drill stall detected");
        }

        // Check how long it has been stalled
        double stall_duration =
            (now - sampler.stall_start_time_).seconds();

        if (stall_duration >= STALL_TIME)
        {
            if (!sampler.drill_stuck_)
            {
                sampler.drill_stuck_ = true;

                RCLCPP_INFO(
                    logger,
                    "DRILL STUCK!");
            }
        }
    }
    else
    {
        // Drill is moving normally again
        sampler.drill_stall_active_ = false;
        sampler.drill_stuck_ = false;
    }

}

}


