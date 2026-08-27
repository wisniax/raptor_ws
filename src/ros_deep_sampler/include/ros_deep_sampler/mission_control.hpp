#pragma once

#include <memory>
#include <string>
#include <vector>
#include <cstdint>

#include <rclcpp/rclcpp.hpp>

#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64.hpp"
#include <cmath>


#include "rex_interfaces/msg/sampler_control.hpp"
#include "rex_interfaces/msg/sampler_feedback.hpp"
#include "rex_interfaces/msg/rover_status.hpp"
#include "ros_deep_sampler/autonomy_controller.hpp"
#include <chrono>
#include <iostream>
#include "ros_deep_sampler/RosCanConstants.hpp"


namespace ros_deep_sampler{

using RoverStatusMsg = rex_interfaces::msg::RoverStatus;

using MissionMsg = rex_interfaces::msg::SamplerFeedback;
using MissionCmd = rex_interfaces::msg::SamplerControl;
//using SamplerCanCmd = sampler_motion_interfaces::msg::SamplerCanCmd;

class MissionControl : public rclcpp::Node
{
public:

    explicit MissionControl(
        const rclcpp::NodeOptions& options);


private:

    // =========================================================
    // Mission-level states
    // =========================================================

    enum class MissionState
    {
        IDLE,
        MANUAL,
        AUTONOMY,
        ABORTED,
        STOPPED
    };


    enum class ControlType
    {
        NO_SAMPLER,
        MANUAL,
        AUTONOMY
    };


    // =========================================================
    // Main mission loop
    // =========================================================

    void MissionUpdate();

    void statesLoop();


    // =========================================================
    // Mission control
    // =========================================================

    void setControlType(
        const RoverStatusMsg::ConstSharedPtr& msg);

    bool isSamplerMode(
        const RoverStatusMsg::ConstSharedPtr& msg);

    void HandleMissionCmd(
        const MissionCmd& missionCmd);

    uint8_t getMissionCmd();

    bool checkCommands(
        uint8_t current_mission_cmd);


    void startMission();

    void stopExecuting();
    void abortExecuting();
    void restartAutonomy();
    void calibrateSampler();



    // =========================================================
    // Execution modes
    // =========================================================

    void executeManual();

    void executeAutonomy();


    // =========================================================
    // ROS callbacks
    // =========================================================

    void HandleRoverStatus(
        const RoverStatusMsg::ConstSharedPtr& roverStatusMsg);

    // void HandleMeasurementFeedback(
    //     const MeasurementMsg::ConstSharedPtr& measurementMsg);

    // =========================================================
    // Manual control
    // =========================================================

    void retranslateSamplerCtrlMsg(const MissionCmd::SharedPtr &missionCmd);


    // =========================================================
    // Feedback
    // =========================================================
    std::string to_string();

    void publishAppFeedback();

    void updateFeedback();

    void getFeedback(MissionMsg::SharedPtr &missionFeedback, JointMovement& joints_);


    // =========================================================
    // Measurements
    // =========================================================

    bool get_measurements();

    int check_drilling();

    bool drillStuck();


    // =========================================================
    // Hardware
    // =========================================================

    void send_rotor_velocity(
        double vel);

    double getPlatformPosition();

    uint8_t to_Feedback();
    uint8_t ctrl_to_Feedback();


private:

    // =========================================================
    // Mission state
    // =========================================================

    MissionState mission_state_ =
        MissionState::IDLE;

    ControlType ctrlType_ =
        ControlType::NO_SAMPLER;


    // =========================================================
    // Mission commands
    // =========================================================

    MissionCmd::SharedPtr missionCmdMsg;
    bool platform_cmd_pending_ = false;
    bool drill_cmd_pending_ = false;
    bool container_cmd_pending_ = false;
    bool clamp_cmd_pending_ = false;

    bool drill_rotor_cmd_pending_ = false;
    bool vacuum_cmd_pending_ = false;
    bool brush_cmd_pending_ = false;

    MissionCmd::SharedPtr LastMissionCmdMsg;

    bool mission_in_stop = false;
    bool autonomy_stopped = false;
    bool new_mission_cmd = false;

    bool was_manual = false;
    const float NO_CMD = -999.0f;

   

    AutonomyController::AutonomyStates state_to_stop_ = AutonomyController::AutonomyStates::IDLE;


    // =========================================================
    // Current sampler state
    // =========================================================

    SamplerState sampler_state_;


    // =========================================================
    // Controllers
    // =========================================================

    AutonomyController autonomy_;


    // =========================================================
    // Hardware
    // =========================================================

    std::unique_ptr<JointMovement> joints_;


    // =========================================================
    // ROS subscriptions
    // =========================================================

    rclcpp::Subscription<RoverStatusMsg>::SharedPtr
        mRoverStatus_;

    rclcpp::Subscription<MissionCmd>::SharedPtr
        mMissionCmd_;

    rclcpp::Subscription<SamplerControlMsg>::SharedPtr
        mSamplerCtrl_;

    // =========================================================
    // ROS publishers
    // =========================================================

    rclcpp::Publisher<MissionMsg>::SharedPtr
        PubFeedback_;

    // =========================================================
    // Other ROS data
    // =========================================================

    std_msgs::msg::String::SharedPtr SubStatus;

    MissionMsg::SharedPtr missionFeedbackMsg;

    // Direct publishing for velocity joint contrl
    // rclcpp::Publisher<SamplerCanCmd>::SharedPtr sampler_can_cmd_pub_;

    // =========================================================
    // Timers
    // =========================================================

    rclcpp::TimerBase::SharedPtr missionTimer_;

    rclcpp::TimerBase::SharedPtr appFeedbackTimer_;


    // =========================================================
    // Mission parameters
    // =========================================================

    static constexpr double TOLERANCE = 0.001;

   


    // =========================================================
    // Control mode values
    // =========================================================

    static constexpr unsigned int CONTROL_MODE_DEEP_SAMPLER = 32;

    static constexpr unsigned int CONTROL_MODE_SURFACE_SAMPLER = 64;

    static constexpr unsigned int
        CONTROL_MODE_DEEP_SAMPLER_AUTONOMY = 512;

    static constexpr unsigned int
        CONTROL_MODE_SURFACE_SAMPLER_AUTONOMY = 1024;


    // =========================================================
    // ROS node
    // =========================================================

    static constexpr const char* NODE_NAME =
        "ros_deep_sampler";
};

}