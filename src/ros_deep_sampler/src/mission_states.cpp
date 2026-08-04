#include "ros_deep_sampler/mission_control.hpp"


namespace ros_deep_sampler{

void MissionControl::statesLoop(){
    // RCLCPP_INFO(this->get_logger(), "statesLoop running");
    uint8_t current_mission_cmd = missionCmdMsg->mission_cmd;
    missionFeedbackMsg->mission_state = to_Feedback(state_);
    getFeedback(missionFeedbackMsg);

    switch(state_){
      case State::IDLE:
        if(!checkCommands(current_mission_cmd)){break;}

         
        if(!calibrate_platform){
          state_ = State::CALIBRATE_PLATFORM;
          mission_in_stop = false;
        }else  if(!calibrate_drill){
          mission_in_stop = false;
          state_ = State::CALIBRATE_DRILL;
        }

        if(calibrate_platform && calibrate_drill){
          state_ = State::MOVE_PLATFORM_DOWN;
          }

            
          
      // waiting for mission_start callback
        break;

      case State::CALIBRATE_PLATFORM:
        if (!checkCommands(current_mission_cmd)){
          break;
        }
       
        if(!goal_sent){
          RCLCPP_INFO(this->get_logger(), "Starting PLATFORM CALIBRATION");
          joints_->calibratePlatform(0.02);
          goal_sent = true;
        }
        
        if(std::fabs(0.0 - joints_->get_current_position(JointMovement::JointsIds::PLATFORM)) < 0.01){
          // if(std::fabs(0.0 - joints_->get_current_position(JointMovement::JointsIds::PLATFORM)) < 0.01 &&             !!!!!!!!!!!!!!!!!
          //     std::fabs(0.0 - joints_->get_current_position(JointMovement::JointsIds::DRILL)) < 0.01 &&               !!!!!!!!!!!!!!!!!!
          //     std::fabs(0.0 - joints_->get_current_position(JointMovement::JointsIds::CONTAINER)) < 0.01){ do other}  !!!!!!!!!!!!!!!!!!!
         
          RCLCPP_INFO(this->get_logger(), "Finishing moving up");
          goal_sent = false;
          calibrate_platform = true;
          joints_->cancelMovement();
       
          joints_->setTrajectoryStatus(false);
          time_between_states = 0;
          state_ = State::MOVE_PLATFORM_DOWN; 
          //}
         
        }
        
      break;

      case State::CALIBRATE_DRILL:
     
        // if (!checkCommands(current_mission_cmd)){
        //   break;
        // }

        // if(!goal_sent){
        //   RCLCPP_INFO(this->get_logger(), "Starting DRILL CALIBRATION");
        //   joints_->calibrateDrill(0.02);
        //   goal_sent = true;
        // }
        
        
        // if(std::fabs(0.0 - joints_->get_current_position(JointMovement::JointsIds::DRILL)) < 0.01){
        //   // time_between_states++;
        //   // if(time_between_states >100){
        //   RCLCPP_INFO(this->get_logger(), "Finishing moving up");
        //   goal_sent = false;
        //   calibrate_drill = true;
        //   joints_->cancelMovement();
        //   //while(joints_->isGoalCanceled() != 1){}
        //   joints_->setTrajectoryStatus(false);
        //   time_between_states = 0;
        //   state_ = State::IDLE; 
        //   //}
         
        // }
        
      break;

      case State::MOVE_PLATFORM_DOWN:
         if (!checkCommands(current_mission_cmd)){
          break;
        }

        if(!goal_sent){
          //joints_->setGoalStatus(false);
          RCLCPP_INFO(this->get_logger(), "Starting moving down");
          JointMovement::JointCommand cmd;
          cmd.id = JointMovement::JointsIds::PLATFORM;
          cmd.start_position = joints_->get_current_position(cmd.id);
          cmd.final_position = -0.4;
          cmd.max_velocity = 0.1;
          commands.push_back(cmd);
  
          cmd.id = JointMovement::JointsIds::DRILL;
          cmd.start_position = joints_->get_current_position(cmd.id);
          cmd.final_position = joints_->get_current_position(cmd.id);
          cmd.max_velocity = 0.2;
          commands.push_back(cmd);

          cmd.id = JointMovement::JointsIds::CONTAINER;
          cmd.start_position = joints_->get_current_position(cmd.id);
          cmd.final_position = joints_->get_current_position(cmd.id);
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
            state_ = State::GET_SURFACE_SAMPLE; 
          }
         
          
        }
        break;

      case State::GET_SURFACE_SAMPLE:
         if (!checkCommands(current_mission_cmd)){
          break;
        }
        if(rotation_time == 0){
            joints_->send_rotor_velocity(JointMovement::JointsIds::VACUUM_ROTOR, 10.0);
            joints_->send_rotor_velocity(JointMovement::JointsIds::BRUSH_ROTOR, 10.0);
        }
        rotation_time ++;
        if(rotation_time > 500 && !goal_sent){
            joints_->send_rotor_velocity(JointMovement::JointsIds::VACUUM_ROTOR, 0.0);
            joints_->send_rotor_velocity(JointMovement::JointsIds::BRUSH_ROTOR, 0.0);
            state_ = State::DRILLING;
            rotation_time =0;
            goal_sent = false;
        }

       
      break;

      case State::DRILLING:
        if (!checkCommands(current_mission_cmd)){
          break;
        }

        if(!goal_sent){
          RCLCPP_INFO(this->get_logger(), "Starting drilling");
          JointMovement::JointCommand cmd;
          cmd.id = JointMovement::JointsIds::PLATFORM;
          cmd.start_position = joints_->get_current_position(cmd.id);
          cmd.final_position = joints_->get_current_position(cmd.id);
          cmd.max_velocity = 0.1;
          commands.push_back(cmd);
  
          cmd.id = JointMovement::JointsIds::DRILL;
          cmd.start_position = joints_->get_current_position(cmd.id);
          cmd.final_position = -0.35;
          cmd.max_velocity = 0.1;
          commands.push_back(cmd);

          cmd.id = JointMovement::JointsIds::CONTAINER;
          cmd.start_position = joints_->get_current_position(cmd.id);
          cmd.final_position = joints_->get_current_position(cmd.id);
          cmd.max_velocity = 0.2;
          commands.push_back(cmd);
          joints_->moveJoints(commands);
          joints_->send_rotor_velocity(JointMovement::JointsIds::DRILL_ROTOR, 15.0);
          goal_sent = true;
        }

        if (drillStuck()){
          RCLCPP_INFO(this->get_logger(), "Drill got stuck");
          joints_->send_rotor_velocity(JointMovement::JointsIds::DRILL_ROTOR, 0.0);
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
            joints_->send_rotor_velocity(JointMovement::JointsIds::DRILL_ROTOR, 0.0);
            recovery_attempt = 0;
            state_ = State::MOVE_DRILL_UP; 
          }
          
         
        }
        break;

      case State::MOVE_DRILL_UP:
           if (!checkCommands(current_mission_cmd)){
            break;
          }
          if(!goal_sent){
            RCLCPP_INFO(this->get_logger(), "Moving drill up");
            JointMovement::JointCommand cmd;
            cmd.id = JointMovement::JointsIds::PLATFORM;
            cmd.start_position = joints_->get_current_position(cmd.id);
            cmd.final_position = joints_->get_current_position(cmd.id);
            cmd.max_velocity = 0.1;
            commands.push_back(cmd);
    
            cmd.id = JointMovement::JointsIds::DRILL;
            cmd.start_position = joints_->get_current_position(cmd.id);
            cmd.final_position = 0.01;
            cmd.max_velocity = 0.1;
            commands.push_back(cmd);

            cmd.id = JointMovement::JointsIds::CONTAINER;
            cmd.start_position = joints_->get_current_position(cmd.id);
            cmd.final_position = joints_->get_current_position(cmd.id);
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
          if (!checkCommands(current_mission_cmd)){
            break;
          }
            if(!goal_sent){
              RCLCPP_INFO(this->get_logger(), "Moving Platform up");
              JointMovement::JointCommand cmd;
              cmd.id = JointMovement::JointsIds::PLATFORM;
              cmd.start_position = joints_->get_current_position(cmd.id);
              cmd.final_position = 0.01;
              cmd.max_velocity = 0.1;
              commands.push_back(cmd);
      
              cmd.id = JointMovement::JointsIds::DRILL;
              cmd.start_position = joints_->get_current_position(cmd.id);
              cmd.final_position = joints_->get_current_position(cmd.id);
              cmd.max_velocity = 0.1;
              commands.push_back(cmd);

              cmd.id = JointMovement::JointsIds::CONTAINER;
              cmd.start_position = joints_->get_current_position(cmd.id);
              cmd.final_position = joints_->get_current_position(cmd.id);
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
                  state_ = State::MOVE_CONTAINER; 
                }
            }   
      break; 

      case State::MOVE_CONTAINER:
           if (!checkCommands(current_mission_cmd)){
            break;
          }
      
          if(!goal_sent){
            RCLCPP_INFO(this->get_logger(), "Starting moving container");
            JointMovement::JointCommand cmd;
            cmd.id = JointMovement::JointsIds::PLATFORM;
            cmd.start_position = joints_->get_current_position(cmd.id);
            cmd.final_position = joints_->get_current_position(cmd.id);
            cmd.max_velocity = 0.1;
            commands.push_back(cmd);
    
            cmd.id = JointMovement::JointsIds::DRILL;
            cmd.start_position = joints_->get_current_position(cmd.id);
            cmd.final_position = joints_->get_current_position(cmd.id);
            cmd.max_velocity = 0.1;
            commands.push_back(cmd);

            cmd.id = JointMovement::JointsIds::CONTAINER;
            cmd.start_position = joints_->get_current_position(cmd.id);
            cmd.final_position = -1.57;
            cmd.max_velocity = 0.6;
            commands.push_back(cmd);
            joints_->moveJoints(commands);
            goal_sent = true;
          }

           if(joints_->isTrajectoryFinished() ==1){
            time_between_states++;
            if(time_between_states > 100){
              time_between_states = 0;
              goal_sent = false;
              joints_->setTrajectoryStatus(false);
              commands.clear();
              state_ = State::MOVE_DRILL_CLOSER; 
              }
            }
        break;

        case State::MOVE_DRILL_CLOSER:
            if (!checkCommands(current_mission_cmd)){
              break;
            }
            if(!goal_sent){
                RCLCPP_INFO(this->get_logger(), "Move drill closr to container");
                JointMovement::JointCommand cmd;
                cmd.id = JointMovement::JointsIds::PLATFORM;
                cmd.start_position = joints_->get_current_position(cmd.id);
                cmd.final_position = joints_->get_current_position(cmd.id);
                cmd.max_velocity = 0.1;
                commands.push_back(cmd);
        
                cmd.id = JointMovement::JointsIds::DRILL;
                cmd.start_position = joints_->get_current_position(cmd.id);
                cmd.final_position = -0.15;
                cmd.max_velocity = 0.1;
                commands.push_back(cmd);

                cmd.id = JointMovement::JointsIds::CONTAINER;
                cmd.start_position = joints_->get_current_position(cmd.id);
                cmd.final_position = joints_->get_current_position(cmd.id);
                cmd.max_velocity = 0.6;
                commands.push_back(cmd);
                joints_->moveJoints(commands);
                // move_client_->send_goal(commands);
                goal_sent = true;
              }
            if(joints_->isTrajectoryFinished() ==1){
              time_between_states++;
              if(time_between_states > 100){
                time_between_states = 0;
                goal_sent = false;
                joints_->setTrajectoryStatus(false);
                commands.clear();
                state_ = State::PUT_DEEP_SAMPLE; 
                }
            }
        break;

        case State::PUT_DEEP_SAMPLE:
            if (!checkCommands(current_mission_cmd)){
                break;
              }
            if(rotation_time == 0){
              joints_->send_rotor_velocity(JointMovement::JointsIds::DRILL_ROTOR, 5.0);
            }
            rotation_time ++;
            if(rotation_time > 300 && !goal_sent){
              joints_->send_rotor_velocity(JointMovement::JointsIds::DRILL_ROTOR, 0.0);
              JointMovement::JointCommand cmd;
              state_ = State::HIDE_DRILL;
              rotation_time =0;
              goal_sent = false;
            }

        break;
          
        case State::HIDE_DRILL:
            if (!checkCommands(current_mission_cmd)){
              break;
            }
            if(!goal_sent){
                RCLCPP_INFO(this->get_logger(), "Move drill closr to container");
                JointMovement::JointCommand cmd;
                cmd.id = JointMovement::JointsIds::PLATFORM;
                cmd.start_position = joints_->get_current_position(cmd.id);
                cmd.final_position = joints_->get_current_position(cmd.id);
                cmd.max_velocity = 0.1;
                commands.push_back(cmd);
        
                cmd.id = JointMovement::JointsIds::DRILL;
                cmd.start_position = joints_->get_current_position(cmd.id);
                cmd.final_position = 0.01;
                cmd.max_velocity = 0.1;
                commands.push_back(cmd);

                cmd.id = JointMovement::JointsIds::CONTAINER;
                cmd.start_position = joints_->get_current_position(cmd.id);
                cmd.final_position = joints_->get_current_position(cmd.id);
                cmd.max_velocity = 0.6;
                commands.push_back(cmd);
                joints_->moveJoints(commands);
                // move_client_->send_goal(commands);
                goal_sent = true;
              }
            if(joints_->isTrajectoryFinished() ==1){
              time_between_states++;
              if(time_between_states > 100){
                time_between_states = 0;
                goal_sent = false;
                joints_->setTrajectoryStatus(false);
                commands.clear();
                state_ = State::MOVE_PLATFORM_CLOSER; 
                }
            }
          break;

        case State::MOVE_PLATFORM_CLOSER:
            if (!checkCommands(current_mission_cmd)){
              break;
            }
            if(!goal_sent){
                RCLCPP_INFO(this->get_logger(), "Move Platform closr to container");
                JointMovement::JointCommand cmd;
                cmd.id = JointMovement::JointsIds::PLATFORM;
                cmd.start_position = joints_->get_current_position(cmd.id);
                cmd.final_position = -0.2;
                cmd.max_velocity = 0.1;
                commands.push_back(cmd);
        
                cmd.id = JointMovement::JointsIds::DRILL;
                cmd.start_position = joints_->get_current_position(cmd.id);
                cmd.final_position = joints_->get_current_position(cmd.id);
                cmd.max_velocity = 0.1;
                commands.push_back(cmd);

                cmd.id = JointMovement::JointsIds::CONTAINER;
                cmd.start_position = joints_->get_current_position(cmd.id);
                cmd.final_position = joints_->get_current_position(cmd.id);
                cmd.max_velocity = 0.6;
                commands.push_back(cmd);
                joints_->moveJoints(commands);
                goal_sent = true;
              }
            if(joints_->isTrajectoryFinished() ==1){
              time_between_states++;
              if(time_between_states > 100){
                time_between_states = 0;
                goal_sent = false;
                joints_->setTrajectoryStatus(false);
                commands.clear();
                state_ = State::PUT_SURFACE_SAMPLE; 
                }
            }
        break;

        case State::PUT_SURFACE_SAMPLE:
            if (!checkCommands(current_mission_cmd)){
                break;
              }
            if(rotation_time == 0){
              joints_->send_rotor_velocity(JointMovement::JointsIds::BRUSH_ROTOR, 5.0);
            }
            rotation_time ++;
            if(rotation_time > 300 && !goal_sent){
              joints_->send_rotor_velocity(JointMovement::JointsIds::BRUSH_ROTOR, 0.0);
              JointMovement::JointCommand cmd;
              state_ = State::MOVE_PLATFORM_BACK;
              rotation_time =0;
              goal_sent = false;
            }

        break;

        case State::MOVE_PLATFORM_BACK:
            if (!checkCommands(current_mission_cmd)){
              break;
            }
            if(!goal_sent){
                RCLCPP_INFO(this->get_logger(), "Move Platform closr to container");
                JointMovement::JointCommand cmd;
                cmd.id = JointMovement::JointsIds::PLATFORM;
                cmd.start_position = joints_->get_current_position(cmd.id);
                cmd.final_position = 0.01;
                cmd.max_velocity = 0.1;
                commands.push_back(cmd);
        
                cmd.id = JointMovement::JointsIds::DRILL;
                cmd.start_position = joints_->get_current_position(cmd.id);
                cmd.final_position = joints_->get_current_position(cmd.id);
                cmd.max_velocity = 0.1;
                commands.push_back(cmd);

                cmd.id = JointMovement::JointsIds::CONTAINER;
                cmd.start_position = joints_->get_current_position(cmd.id);
                cmd.final_position = 0.01;
                cmd.max_velocity = 0.6;
                commands.push_back(cmd);
                joints_->moveJoints(commands);
                // move_client_->send_goal(commands);
                goal_sent = true;
              }
            if(joints_->isTrajectoryFinished() ==1){
              time_between_states++;
              if(time_between_states > 100){
                time_between_states = 0;
                goal_sent = false;
                joints_->setTrajectoryStatus(false);
                commands.clear();
                state_ = State::MEASURE_SAMPLES; 
                }
            }
        break;

        // case State::HIDE_CONTAINER:
        //     if (!checkCommands(current_mission_cmd)){
        //       break;
        //     }
        //     if(!goal_sent){
        //         RCLCPP_INFO(this->get_logger(), "Move drill closr to container");
        //         JointMovement::JointCommand cmd;
        //         cmd.id = JointMovement::JointsIds::CONTAINER;
        //         cmd.position = 0.01;
        //         cmd.max_velocity = 0.6;
        //         commands.push_back(cmd);
        //         joints_->moveJoints(commands);
        //         goal_sent = true;
        //       }
        //     if(joints_->isTrajectoryFinished() ==1){
        //       time_between_states++;
        //       if(time_between_states > 100){
        //         time_between_states = 0;
        //         goal_sent = false;
        //         joints_->setTrajectoryStatus(false);
        //         commands.clear();
        //         state_ = State::MEASURE_SAMPLES; 
        //         }
        //     }
        // break;

        case State::MEASURE_SAMPLES:
            if (!checkCommands(current_mission_cmd)){
              break;
            }
            if(get_measurements()){
              state_ = State::DONE;
            }
        break;
          

      
        case State::DONE:
        if(!checkCommands(current_mission_cmd)){
          break;
        }

        if (ctrlType_ == CONTROL_TYPE::AUTONOMY && current_mission_cmd == MissionCmd::START){
          calibrate_drill = false;
          calibrate_platform = false;
          state_ = State::IDLE;
        }
        break;

        case State::MANUAL_CONTROL:
          if(!checkCommands(current_mission_cmd)){
            break;
          }

          retranslateSamplerCtrlMsg(missionCmdMsg);

        break;

        case State::RECOVER_DRILL:
          // if (!checkCommands(current_mission_cmd)){
          //   break;
          // }
          // if (recovery_attempt > MAX_RECOVERY_ATTEMPT){
          //     state_to_abort = state_;
          //     state_ = State::ABORT;
          //   }

          // switch(recover_state){
          //   case RECOVER_STATE::LIFT_UP:
          //   if(!goal_sent){
          //     RCLCPP_INFO(this->get_logger(), "Lift drill a bit up to recover");
          //     JointMovement::JointCommand cmd;
          //     cmd.id = JointMovement::JointsIds::DRILL;
          //     cmd.position = joints_->get_current_position(cmd.id) + 0.05;
          //     cmd.max_velocity = 0.1;
          //     commands.push_back(cmd);
          //     joints_->moveJoints(commands);
          //     joints_->send_rotor_velocity(JointMovement::JointsIds::DRILL_ROTOR, -5.0);
          //     goal_sent = true;
          //     stall_timer_running_ = false;
          //   }

          //   if(joints_->isTrajectoryFinished() ==1){
          //     time_between_states++;
          //     if(time_between_states > 50){
          //       RCLCPP_INFO(this->get_logger(), "Finishing moving up");
          //       time_between_states = 0;
          //       goal_sent = false;
          //       joints_->setTrajectoryStatus(false);
          //       commands.clear();
          //       recover_state = RECOVER_STATE::WAIT;
          //       break;
          //     }
          //   }
          //   break;

          //   case RECOVER_STATE::WAIT:
          //     rotation_time++;
          //     if(rotation_time>10){
          //        joints_->send_rotor_velocity(JointMovement::JointsIds::DRILL_ROTOR, 12.0);
          //        stall_timer_running_ = false;
          //        rotation_time = 0;
          //        recover_state = RECOVER_STATE::CHECK_ROTATION;
          //     }
          //   break;
          //   case RECOVER_STATE::CHECK_ROTATION:
          //     time_between_states++;
          //     if(time_between_states > 100){
          //       time_between_states =0;
          //       if (drillStuck()){
          //         joints_->send_rotor_velocity(JointMovement::JointsIds::DRILL_ROTOR, 0.0);
          //         goal_sent = false;
          //         recover_state = RECOVER_STATE::LIFT_UP;
          //         recovery_attempt++;
          //       }else{
          //         joints_->send_rotor_velocity(JointMovement::JointsIds::DRILL_ROTOR, 0.0);
          //         goal_sent = false;
          //         recover_state = RECOVER_STATE::LIFT_UP;
          //         recovery_attempt = 0;
          //         state_ = State::DRILLING;
          //         break;
          //       }
          //     }

          //   break;
          // }
          

        break;

        case State::STOP:
          if(!mission_in_stop){
            std::string state_string = to_string(state_to_stop);
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
            state_= State::STOP;
          }
          if(checkCommands(current_mission_cmd) && current_mission_cmd == MissionCmd::START){
            state_ = state_to_stop;
            std::string state_string = to_string(state_to_stop);
            RCLCPP_INFO(this->get_logger(), "Continue the mission from state: %s", state_string.c_str());
            mission_in_stop = false;
          }
        
        break;

        case State::ABORT:
            joints_->send_rotor_velocity(JointMovement::JointsIds::DRILL_ROTOR, 0.0);
            std::string state_string = to_string(state_to_abort);
            RCLCPP_INFO(this->get_logger(), "Mission Abortion on state: %s", state_string.c_str());
            //move_client_->cancel_goal();
            joints_->cancelMovement();
            // while(joints_->isGoalCanceled() != 1){}
            RCLCPP_INFO(this->get_logger(), "Mission Aborted successfully");
            state_= State::MANUAL_CONTROL;

        break;
    }
}



}