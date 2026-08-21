#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "rex_interfaces/msg/rover_status.hpp"
#include "rex_interfaces/msg/sampler_control.hpp"

#include "ros_deep_sampler/joint_movement.hpp"
#include "sampler_motion_interfaces/msg/sampler_mission.hpp"
#include "sampler_motion_interfaces/msg/sampler_mission_cmd.hpp"

namespace ros_deep_sampler{

using RoverStatusMsg = rex_interfaces::msg::RoverStatus;
using SamplerControlMsg = rex_interfaces::msg::SamplerControl;
using MissionMsg = sampler_motion_interfaces::msg::SamplerMission;
using MissionCmd = sampler_motion_interfaces::msg::SamplerMissionCmd;

struct SamplerState
{
    double platform_position = -100.0;
    double drill_position = -100.0;
    double rotor_velocity = 0.0;
    bool rotor_in_action = false;

    rclcpp::Time stall_start_time_;
    bool drill_stall_active_ = false;
    bool drill_stuck_ = false;

    double container_pos = 0.0;
    double brush_vel = 0.0;
    double vacuum_vel = 0.0;
    double distance_to_ground = -999.0;

    RoverStatusMsg::ConstSharedPtr rover_status;
    SamplerControlMsg::ConstSharedPtr sampler_control;
};


class AutonomyController
{
public:

    enum class AutonomyStates
    {
        IDLE,
        CALIBRATE_JOINTS,

        MOVE_PLATFORM_DOWN,
        GET_SURFACE_SAMPLE,
        DRILLING,
        RECOVER_DRILL,

        MOVE_DRILL_UP,
        MOVE_PLATFORM_UP,

        MOVE_CONTAINER,
        MOVE_DRILL_CLOSER,
        PUT_DEEP_SAMPLE,
        HIDE_DRILL,

        MOVE_PLATFORM_CLOSER,
        PUT_SURFACE_SAMPLE,
        MOVE_PLATFORM_BACK,
        HIDE_CONTAINER,

        MEASURE_SAMPLES,
        DONE,
        ABORT
    };

    enum class RecoverState
    {
        LIFT_UP,
        WAIT
    };


    AutonomyController() = default;


    void executeAutonomy(SamplerState& sampler,
                        const MissionCmd& missionCmd,
                        MissionMsg& missionFeedback,
                        JointMovement& joints_,
                        const rclcpp::Logger& logger);


    void setState(AutonomyStates state);
    void resetStateVariables();

    void stop(JointMovement& joints_);

    void abort();

    void restart();

    void requestCalibration(JointMovement& joints_,
                        const rclcpp::Logger& logger);

    // void getFeedback(MissionMsg& missionFeedback,
    //                 JointMovement& joints_);


    AutonomyStates getState() const;
    bool getCalibrationState();

    std::string toString(AutonomyStates state) const;

    uint8_t toFeedback(AutonomyStates state) const;


private:

    void executeIdle(const SamplerState& sampler);

    void executeRecovery(const SamplerState& sampler);

    void checkStall(SamplerState& sampler,
                    const rclcpp::Time& now,
                    const rclcpp::Logger& logger);

    void handleStall();


private:

    AutonomyStates state_ =
        AutonomyStates::IDLE;

    AutonomyStates state_to_abort_ =
        AutonomyStates::IDLE;

    AutonomyStates state_to_stop_ =
        AutonomyStates::IDLE;

    AutonomyStates state_to_recover_ =
        AutonomyStates::IDLE;


    //bool stall_timer_running_ = false;


    int recovery_attempt = 0;

    RecoverState recover_state_ =
        RecoverState::LIFT_UP;


    static constexpr double MIN_SPEED = 3.0;
    static constexpr double MAX_CURRENT = 3.0;
    static constexpr double MIN_TIME = 0.5;
    static constexpr double STALL_TIME = 1.0;               // seconds

    static constexpr int MAX_RECOVERY_ATTEMPT = 5;


    bool goal_sent = false;
    int rotation_time = 0;
    int time_between_states = 100;


    bool calibrate_drill = false;
    bool calibrate_platform = false;


    std::vector<JointMovement::JointCommand> commands;
};
}