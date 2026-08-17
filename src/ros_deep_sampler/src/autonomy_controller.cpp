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

}


