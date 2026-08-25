#include "ros_deep_sampler/autonomy_controller.hpp"


namespace ros_deep_sampler{


void AutonomyController::executeAutonomy(SamplerState& sampler, const MissionCmd& missionCmd,
                                          MissionMsg& missionFeedback,
                                          JointMovement& joints_,
                                          const rclcpp::Logger& logger){
    // RCLCPP_INFO(logger, "statesLoop running");
    uint8_t current_mission_cmd = missionCmd.mission_cmd;
    // missionFeedbac.mission_state = to_Feedback(state_);
    //getFeedback(missionFeedback, joints_);
    sampler.rotor_velocity = joints_.get_current_velocity(JointMovement::JointsIds::DRILL_ROTOR);
    rclcpp::Time now = rclcpp::Clock().now();
    checkStall(sampler, now, logger);
    switch(state_){
      case AutonomyStates::IDLE:

         
        if(!calibrate_platform){
          state_ = AutonomyStates::CALIBRATE_JOINTS;
        }

            
          
      // waiting for mission_start callback
      break;

      case AutonomyStates::CALIBRATE_JOINTS:
       
        if(!goal_sent){
          RCLCPP_INFO(logger, "Starting JOINTS CALIBRATION");
          joints_.stopMotors();
          joints_.calibrateJoints(0.02);
          goal_sent = true;
        }
        
        if(std::fabs(0.0 - joints_.get_current_position(JointMovement::JointsIds::PLATFORM)) < 0.01){
          // if(std::fabs(0.0 - joints_.get_current_position(JointMovement::JointsIds::PLATFORM)) < 0.01 &&             !!!!!!!!!!!!!!!!!
          //     std::fabs(0.0 - joints_.get_current_position(JointMovement::JointsIds::DRILL)) < 0.01 &&               !!!!!!!!!!!!!!!!!!
          //     std::fabs(0.0 - joints_.get_current_position(JointMovement::JointsIds::CONTAINER)) < 0.01){ do other}  !!!!!!!!!!!!!!!!!!!
          //      OR BASED ON SWITCHES FEEDBACK IF AVAILIABLE
          //if(joints_.get_end_switch_state(JointMovement::JointsIds::PLATFORM && 
          //   joints_.get_end_switch_state(JointMovement::JointsIds::DRILL &&
          //   joints_.get_end_switch_state(JointMovement::JointsIds::CONTAINER){}        
          RCLCPP_INFO(logger, "Finishing moving up");
          goal_sent = false;
          calibrate_platform = true;
          joints_.cancelMovement();
       
          joints_.setTrajectoryStatus(false);
          time_between_states = 0;
          sampler.distance_to_ground = joints_.get_distance_to_ground();
          state_ = AutonomyStates::MOVE_PLATFORM_DOWN; 
          //}
         
        }
        
      break;

      case AutonomyStates::MOVE_PLATFORM_DOWN:

        if(!goal_sent){
          joints_.stopMotors();
          //joints_.setGoalStatus(false);
          RCLCPP_INFO(logger, "Starting moving down");
          JointMovement::JointCommand cmd;
          cmd.id = JointMovement::JointsIds::PLATFORM;
          cmd.start_position = joints_.get_current_position(cmd.id);
          cmd.final_position = -1* sampler.distance_to_ground;
          cmd.max_velocity = 0.1;
          commands.push_back(cmd);
  
          cmd.id = JointMovement::JointsIds::DRILL;
          cmd.start_position = joints_.get_current_position(cmd.id);
          cmd.final_position = joints_.get_current_position(cmd.id);
          cmd.max_velocity = 0.2;
          commands.push_back(cmd);

          cmd.id = JointMovement::JointsIds::CONTAINER;
          cmd.start_position = joints_.get_current_position(cmd.id);
          cmd.final_position = joints_.get_current_position(cmd.id);
          cmd.max_velocity = 0.2;
          commands.push_back(cmd);

          joints_.moveJoints(commands);
          goal_sent = true;
        }
        
        if(joints_.isTrajectoryFinished() == 1){
          time_between_states++;
          if(time_between_states > 50){
            RCLCPP_INFO(logger, "Finishing moving down");
            time_between_states = 0;
            goal_sent = false;
            //move_client_->set_goal_status(goal_sent);
            commands.clear();
            state_ = AutonomyStates::GET_SURFACE_SAMPLE; 
          }
         
          
        }
        break;

      case AutonomyStates::GET_SURFACE_SAMPLE:
        if(rotation_time == 0){
            joints_.stopMotors();
            joints_.send_rotor_velocity(JointMovement::JointsIds::VACUUM_ROTOR, 10.0);
            joints_.send_rotor_velocity(JointMovement::JointsIds::BRUSH_ROTOR, 10.0);
            goal_sent = true;
        }
        rotation_time ++;
        if(rotation_time > 200 && goal_sent){
            joints_.send_rotor_velocity(JointMovement::JointsIds::VACUUM_ROTOR, 0.0);
            joints_.send_rotor_velocity(JointMovement::JointsIds::BRUSH_ROTOR, 0.0);
            state_ = AutonomyStates::DRILLING;
            rotation_time =0;
            goal_sent = false;
        }

       
      break;

      case AutonomyStates::DRILLING:

        if(!goal_sent){
          joints_.stopMotors();
          RCLCPP_INFO(logger, "Starting drilling");
          JointMovement::JointCommand cmd;
          cmd.id = JointMovement::JointsIds::PLATFORM;
          cmd.start_position = joints_.get_current_position(cmd.id);
          cmd.final_position = joints_.get_current_position(cmd.id);
          cmd.max_velocity = 0.1;
          commands.push_back(cmd);
  
          cmd.id = JointMovement::JointsIds::DRILL;
          cmd.start_position = joints_.get_current_position(cmd.id);
          cmd.final_position = -0.35;
          cmd.max_velocity = 0.1;
          commands.push_back(cmd);

          cmd.id = JointMovement::JointsIds::CONTAINER;
          cmd.start_position = joints_.get_current_position(cmd.id);
          cmd.final_position = joints_.get_current_position(cmd.id);
          cmd.max_velocity = 0.2;
          commands.push_back(cmd);
          joints_.moveJoints(commands);
          joints_.send_rotor_velocity(JointMovement::JointsIds::DRILL_ROTOR, 15.0);
          sampler.rotor_in_action = true;
          goal_sent = true;
        }
        

        if (sampler.drill_stuck_){
          RCLCPP_INFO(logger, "Drill got stuck");
          joints_.stopMotors();
          joints_.cancelMovement();
          joints_.setTrajectoryStatus(false);
          time_between_states = 0;
          goal_sent = false;
          commands.clear();
          state_to_recover_ = state_;
          state_ = AutonomyStates::RECOVER_DRILL;
          break;
        }
        
        if(joints_.isTrajectoryFinished() ==1){
          time_between_states++;
          if(time_between_states > 50){
            RCLCPP_INFO(logger, "Finishing moving down during drillling");
            time_between_states = 0;
            goal_sent = false;
            // move_client_->set_goal_status(goal_sent);
            commands.clear();
            joints_.send_rotor_velocity(JointMovement::JointsIds::DRILL_ROTOR, 0.0);
            recovery_attempt = 0;
            sampler.rotor_in_action = false;
            state_ = AutonomyStates::MOVE_DRILL_UP; 
          }
          
         
        }
      break;

      case AutonomyStates::MOVE_DRILL_UP:
          if(!goal_sent){
            joints_.stopMotors();
            RCLCPP_INFO(logger, "Moving drill up");
            JointMovement::JointCommand cmd;
            cmd.id = JointMovement::JointsIds::PLATFORM;
            cmd.start_position = joints_.get_current_position(cmd.id);
            cmd.final_position = joints_.get_current_position(cmd.id);
            cmd.max_velocity = 0.1;
            commands.push_back(cmd);
    
            cmd.id = JointMovement::JointsIds::DRILL;
            cmd.start_position = joints_.get_current_position(cmd.id);
            cmd.final_position = 0.01;
            cmd.max_velocity = 0.1;
            commands.push_back(cmd);

            cmd.id = JointMovement::JointsIds::CONTAINER;
            cmd.start_position = joints_.get_current_position(cmd.id);
            cmd.final_position = joints_.get_current_position(cmd.id);
            cmd.max_velocity = 0.2;
            commands.push_back(cmd);
            joints_.moveJoints(commands);
            goal_sent = true;
          }
          
          if(joints_.isTrajectoryFinished() == 1){
            time_between_states++;
            if(time_between_states > 50){
                RCLCPP_INFO(logger, "Finish moving drill up");
                time_between_states =0;
                goal_sent = false;
                // move_client_->set_goal_status(goal_sent);
                commands.clear();
                state_ = AutonomyStates::MOVE_PLATFORM_UP; 
              }
          }   
      break; 
        
      case AutonomyStates::MOVE_PLATFORM_UP:
            if(!goal_sent){
              joints_.stopMotors();
              RCLCPP_INFO(logger, "Moving Platform up");
              JointMovement::JointCommand cmd;
              cmd.id = JointMovement::JointsIds::PLATFORM;
              cmd.start_position = joints_.get_current_position(cmd.id);
              cmd.final_position = 0.01;
              cmd.max_velocity = 0.1;
              commands.push_back(cmd);
      
              cmd.id = JointMovement::JointsIds::DRILL;
              cmd.start_position = joints_.get_current_position(cmd.id);
              cmd.final_position = joints_.get_current_position(cmd.id);
              cmd.max_velocity = 0.1;
              commands.push_back(cmd);

              cmd.id = JointMovement::JointsIds::CONTAINER;
              cmd.start_position = joints_.get_current_position(cmd.id);
              cmd.final_position = joints_.get_current_position(cmd.id);
              cmd.max_velocity = 0.2;
              commands.push_back(cmd);
              joints_.moveJoints(commands);
              goal_sent = true;
            }
            
            if(joints_.isTrajectoryFinished()  ==1){
              time_between_states++;
              if(time_between_states > 50){
                  RCLCPP_INFO(logger, "Finish moving Platform up");
                  time_between_states =0;
                  goal_sent = false;
                  commands.clear();
                  joints_.setTrajectoryStatus(false);
                  state_ = AutonomyStates::MOVE_CONTAINER; 
                }
            }   
      break; 

      case AutonomyStates::MOVE_CONTAINER:
          if(!goal_sent){
            joints_.stopMotors();
            RCLCPP_INFO(logger, "Starting moving container");
            JointMovement::JointCommand cmd;
            cmd.id = JointMovement::JointsIds::PLATFORM;
            cmd.start_position = joints_.get_current_position(cmd.id);
            cmd.final_position = joints_.get_current_position(cmd.id);
            cmd.max_velocity = 0.1;
            commands.push_back(cmd);
    
            cmd.id = JointMovement::JointsIds::DRILL;
            cmd.start_position = joints_.get_current_position(cmd.id);
            cmd.final_position = joints_.get_current_position(cmd.id);
            cmd.max_velocity = 0.1;
            commands.push_back(cmd);

            cmd.id = JointMovement::JointsIds::CONTAINER;
            cmd.start_position = joints_.get_current_position(cmd.id);
            cmd.final_position = -1.57;
            cmd.max_velocity = 0.6;
            commands.push_back(cmd);
            joints_.moveJoints(commands);
            goal_sent = true;
          }

           if(joints_.isTrajectoryFinished() ==1){
            time_between_states++;
            if(time_between_states > 50){
              time_between_states = 0;
              goal_sent = false;
              joints_.setTrajectoryStatus(false);
              commands.clear();
              state_ = AutonomyStates::MOVE_DRILL_CLOSER; 
              }
            }
        break;

        case AutonomyStates::MOVE_DRILL_CLOSER:
            if(!goal_sent){
                joints_.stopMotors();
                RCLCPP_INFO(logger, "Move drill closr to container");
                JointMovement::JointCommand cmd;
                cmd.id = JointMovement::JointsIds::PLATFORM;
                cmd.start_position = joints_.get_current_position(cmd.id);
                cmd.final_position = joints_.get_current_position(cmd.id);
                cmd.max_velocity = 0.1;
                commands.push_back(cmd);
        
                cmd.id = JointMovement::JointsIds::DRILL;
                cmd.start_position = joints_.get_current_position(cmd.id);
                cmd.final_position = -0.15;
                cmd.max_velocity = 0.1;
                commands.push_back(cmd);

                cmd.id = JointMovement::JointsIds::CONTAINER;
                cmd.start_position = joints_.get_current_position(cmd.id);
                cmd.final_position = joints_.get_current_position(cmd.id);
                cmd.max_velocity = 0.6;
                commands.push_back(cmd);
                joints_.moveJoints(commands);
                // move_client_->send_goal(commands);
                goal_sent = true;
              }
            if(joints_.isTrajectoryFinished() ==1){
              time_between_states++;
              if(time_between_states > 50){
                time_between_states = 0;
                goal_sent = false;
                joints_.setTrajectoryStatus(false);
                commands.clear();
                state_ = AutonomyStates::PUT_DEEP_SAMPLE; 
                }
            }
        break;

        case AutonomyStates::PUT_DEEP_SAMPLE:
            if(rotation_time == 0){
              joints_.stopMotors();
              joints_.send_rotor_velocity(JointMovement::JointsIds::DRILL_ROTOR, 5.0);
              goal_sent = true;
            }
            rotation_time ++;
            if(rotation_time > 70 && goal_sent){
              joints_.send_rotor_velocity(JointMovement::JointsIds::DRILL_ROTOR, 0.0);
              JointMovement::JointCommand cmd;
              state_ = AutonomyStates::HIDE_DRILL;
              rotation_time =0;
              goal_sent = false;
            }

        break;
          
        case AutonomyStates::HIDE_DRILL:
            if(!goal_sent){
                joints_.stopMotors();
                RCLCPP_INFO(logger, "Move drill closr to container");
                JointMovement::JointCommand cmd;
                cmd.id = JointMovement::JointsIds::PLATFORM;
                cmd.start_position = joints_.get_current_position(cmd.id);
                cmd.final_position = joints_.get_current_position(cmd.id);
                cmd.max_velocity = 0.1;
                commands.push_back(cmd);
        
                cmd.id = JointMovement::JointsIds::DRILL;
                cmd.start_position = joints_.get_current_position(cmd.id);
                cmd.final_position = 0.01;
                cmd.max_velocity = 0.1;
                commands.push_back(cmd);

                cmd.id = JointMovement::JointsIds::CONTAINER;
                cmd.start_position = joints_.get_current_position(cmd.id);
                cmd.final_position = joints_.get_current_position(cmd.id);
                cmd.max_velocity = 0.6;
                commands.push_back(cmd);
                joints_.moveJoints(commands);
                // move_client_->send_goal(commands);
                goal_sent = true;
              }
            if(joints_.isTrajectoryFinished() ==1){
              time_between_states++;
              if(time_between_states > 50){
                time_between_states = 0;
                goal_sent = false;
                joints_.setTrajectoryStatus(false);
                commands.clear();
                state_ = AutonomyStates::MOVE_PLATFORM_CLOSER; 
                }
            }
          break;

        case AutonomyStates::MOVE_PLATFORM_CLOSER:
            if(!goal_sent){
                joints_.stopMotors();
                RCLCPP_INFO(logger, "Move Platform closr to container");
                JointMovement::JointCommand cmd;
                cmd.id = JointMovement::JointsIds::PLATFORM;
                cmd.start_position = joints_.get_current_position(cmd.id);
                cmd.final_position = -0.2;
                cmd.max_velocity = 0.1;
                commands.push_back(cmd);
        
                cmd.id = JointMovement::JointsIds::DRILL;
                cmd.start_position = joints_.get_current_position(cmd.id);
                cmd.final_position = joints_.get_current_position(cmd.id);
                cmd.max_velocity = 0.1;
                commands.push_back(cmd);

                cmd.id = JointMovement::JointsIds::CONTAINER;
                cmd.start_position = joints_.get_current_position(cmd.id);
                cmd.final_position = -1.45;
                cmd.max_velocity = 0.6;
                commands.push_back(cmd);
                joints_.moveJoints(commands);
                goal_sent = true;
              }
            if(joints_.isTrajectoryFinished() ==1){
              time_between_states++;
              if(time_between_states > 50){
                time_between_states = 0;
                goal_sent = false;
                joints_.setTrajectoryStatus(false);
                commands.clear();
                state_ = AutonomyStates::PUT_SURFACE_SAMPLE; 
                }
            }
        break;

        case AutonomyStates::PUT_SURFACE_SAMPLE:
            if(rotation_time == 0){
              joints_.stopMotors();
              //joints_.send_rotor_velocity(JointMovement::JointsIds::BRUSH_ROTOR, 5.0);
              joints_.open_clamp();
              goal_sent = true;
            }
            rotation_time ++;
            if(rotation_time > 100 && goal_sent){
              //joints_.send_rotor_velocity(JointMovement::JointsIds::BRUSH_ROTOR, 0.0);
              joints_.close_clamp();
              JointMovement::JointCommand cmd;
              state_ = AutonomyStates::MOVE_PLATFORM_BACK;
              rotation_time =0;
              goal_sent = false;
            }

        break;

        case AutonomyStates::MOVE_PLATFORM_BACK:
            if(!goal_sent){
                joints_.stopMotors();
                RCLCPP_INFO(logger, "Move Platform closr to container");
                JointMovement::JointCommand cmd;
                cmd.id = JointMovement::JointsIds::PLATFORM;
                cmd.start_position = joints_.get_current_position(cmd.id);
                cmd.final_position = 0.01;
                cmd.max_velocity = 0.1;
                commands.push_back(cmd);
        
                cmd.id = JointMovement::JointsIds::DRILL;
                cmd.start_position = joints_.get_current_position(cmd.id);
                cmd.final_position = joints_.get_current_position(cmd.id);
                cmd.max_velocity = 0.1;
                commands.push_back(cmd);

                cmd.id = JointMovement::JointsIds::CONTAINER;
                cmd.start_position = joints_.get_current_position(cmd.id);
                cmd.final_position = 0.01;
                cmd.max_velocity = 0.6;
                commands.push_back(cmd);
                joints_.moveJoints(commands);
                // move_client_->send_goal(commands);
                goal_sent = true;
              }
            if(joints_.isTrajectoryFinished() ==1){
              time_between_states++;
              if(time_between_states > 50){
                time_between_states = 0;
                goal_sent = false;
                joints_.setTrajectoryStatus(false);
                commands.clear();
                state_ = AutonomyStates::DONE; 
                }
            }
        break;

        // case AutonomyStates::HIDE_CONTAINER:
        //     if (!checkCommands(current_mission_cmd)){
        //       break;
        //     }
        //     if(!goal_sent){
        //         RCLCPP_INFO(logger, "Move drill closr to container");
        //         JointMovement::JointCommand cmd;
        //         cmd.id = JointMovement::JointsIds::CONTAINER;
        //         cmd.position = 0.01;
        //         cmd.max_velocity = 0.6;
        //         commands.push_back(cmd);
        //         joints_.moveJoints(commands);
        //         goal_sent = true;
        //       }
        //     if(joints_.isTrajectoryFinished() ==1){
        //       time_between_states++;
        //       if(time_between_states > 100){
        //         time_between_states = 0;
        //         goal_sent = false;
        //         joints_.setTrajectoryStatus(false);
        //         commands.clear();
        //         state_ = AutonomyStates::MEASURE_SAMPLES; 
        //         }
        //     }
        // break;

      //   case AutonomyStates::MEASURE_SAMPLES:
      //       // if (!checkCommands(current_mission_cmd)){
      //       //   break;
      //       // }
      //       // if(get_measurements()){
      //       //   state_ = AutonomyStates::DONE;
      //       // }
      //   break;
          

      
        case AutonomyStates::DONE:
        
        // if (ctrlType_ == ControlType::AUTONOMY && current_mission_cmd == MissionCmd::START){
        //   calibrate_drill = false;
        //   calibrate_platform = false;
        //   state_ = AutonomyStates::IDLE;
        // }
        break;

        case AutonomyStates::ABORT:
            
        break;

        case AutonomyStates::RECOVER_DRILL:
          if (recovery_attempt > MAX_RECOVERY_ATTEMPT){
              //state_to_abort_ = state_;
              stop(joints_);
              resetStateVariables();
              state_ = AutonomyStates::ABORT;
              break;
            }

          switch(recover_state_){
            case RecoverState::LIFT_UP:
            if(!goal_sent){
              RCLCPP_INFO(logger, "Lift drill a bit up to recover");
              JointMovement::JointCommand cmd;
              cmd.id = JointMovement::JointsIds::PLATFORM;
              cmd.start_position = joints_.get_current_position(cmd.id);
              cmd.final_position = joints_.get_current_position(cmd.id);
              cmd.max_velocity = 0.1;
              commands.push_back(cmd);
      
              cmd.id = JointMovement::JointsIds::DRILL;
              cmd.start_position = joints_.get_current_position(cmd.id);
              cmd.final_position = joints_.get_current_position(cmd.id) + 0.01;
              cmd.max_velocity = 0.1;
              commands.push_back(cmd);

              cmd.id = JointMovement::JointsIds::CONTAINER;
              cmd.start_position = joints_.get_current_position(cmd.id);
              cmd.final_position = joints_.get_current_position(cmd.id);
              cmd.max_velocity = 0.6;
              commands.push_back(cmd);
              joints_.moveJoints(commands);

              joints_.send_rotor_velocity(JointMovement::JointsIds::DRILL_ROTOR, -6.0);
              // move_client_->send_goal(commands);
              goal_sent = true;
            }

            if(joints_.isTrajectoryFinished() ==1){
              time_between_states++;
              if(time_between_states > 50){
                RCLCPP_INFO(logger, "Finishing moving up");
                time_between_states = 0;
                rotation_time = 0;
                goal_sent = false;
                joints_.setTrajectoryStatus(false);
                joints_.stopMotors();
                commands.clear();
                recover_state_ = RecoverState::WAIT;
                break;
              }
            }
            break;

            case RecoverState::WAIT:

                rotation_time++;

                joints_.send_rotor_velocity(JointMovement::JointsIds::DRILL_ROTOR, 6.0);

                if (rotation_time > 50) {

                    if (sampler.drill_stuck_) {

                        RCLCPP_WARN(
                            logger,
                            "Drill is still stuck");

                        joints_.stopMotors();

                        recovery_attempt++;

                        rotation_time = 0;
                        goal_sent = false;
                        commands.clear();

                        recover_state_ = RecoverState::LIFT_UP;

                    } else {

                        RCLCPP_INFO(
                            logger,
                            "Drill recovered");

                        joints_.stopMotors();

                        rotation_time = 0;
                        goal_sent = false;
                        commands.clear();

                        state_ = state_to_recover_;
                    }
                }

            break;
          }
          

        break;

      //   case AutonomyStates::STOP:
      //     if(!mission_in_stop){
      //       std::string state_string = to_string(state_to_stop_);
      //       RCLCPP_INFO(logger, "Mission STOP on state: %s", state_string.c_str());
      //       joints_.send_rotor_velocity(JointMovement::JointsIds::DRILL_ROTOR, 0.0);
      //       //move_client_->cancel_goal();
      //       if (goal_sent){
      //         joints_.cancelMovement();
      //         goal_sent = false;
      //       }
      //       // while(joints_.isGoalCanceled() != 1){}
      //       mission_in_stop = true;
      //       RCLCPP_INFO(logger, "Mission Stopped successfully");
      //       state_= AutonomyStates::STOP;
      //     }
      //     if(checkCommands(current_mission_cmd) && current_mission_cmd == MissionCmd::START){
      //       state_ = state_to_stop_;
      //       std::string state_string = to_string(state_to_stop_);
      //       RCLCPP_INFO(logger, "Continue the mission from state: %s", state_string.c_str());
      //       mission_in_stop = false;
      //     }
        
      //   break;

      //   case AutonomyStates::ABORT:
      //       joints_.send_rotor_velocity(JointMovement::JointsIds::DRILL_ROTOR, 0.0);
      //       std::string state_string = to_string(state_to_abort_);
      //       RCLCPP_INFO(logger, "Mission Abortion on state: %s", state_string.c_str());
      //       //move_client_->cancel_goal();
      //       joints_.cancelMovement();
      //       // while(joints_.isGoalCanceled() != 1){}
      //       RCLCPP_INFO(logger, "Mission Aborted successfully");
      //       state_= AutonomyStates::MANUAL_CONTROL;

      //   break;
    }
}
}


