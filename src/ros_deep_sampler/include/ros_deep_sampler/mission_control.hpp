#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "rex_interfaces/msg/sampler_control.hpp"
#include "rex_interfaces/msg/sampler_feedback.hpp"
#include "rex_interfaces/msg/rover_status.hpp"
#include "sampler_motion_interfaces/msg/sampler_mission.hpp"
#include "sampler_motion_interfaces/msg/sampler_mission_cmd.hpp"
#include <chrono>
#include <string>
#include <iostream>
#include "ros_deep_sampler/RosCanConstants.hpp"
#include "ros_deep_sampler/joint_movement.hpp"


namespace ros_deep_sampler{

using SamplerControlMsg = rex_interfaces::msg::SamplerControl;
using RoverStatusMsg = rex_interfaces::msg::RoverStatus;
using MeasurementMsg = rex_interfaces::msg::SamplerFeedback;
using MissionMsg = sampler_motion_interfaces::msg::SamplerMission;
using MissionCmd = sampler_motion_interfaces::msg::SamplerMissionCmd;

class MissionControl : public rclcpp::Node{
    public:
        explicit MissionControl(const rclcpp::NodeOptions & options);
         

        int check_drilling();

        void MissionCheck(std_msgs::msg::String::SharedPtr msg);
        
        bool checkCommands(uint8_t current_mission_cmd);


        void publishAppFeedback();
        void getFeedback(MissionMsg::SharedPtr &feedbackMsg);
        void startMission();
        
        
        bool isSamplerMode(const RoverStatusMsg::ConstSharedPtr &msg);
        void HandleRoverStatus(const RoverStatusMsg::ConstSharedPtr &roverStatusMsg);
        void HandleMeasurementFeedback(const MeasurementMsg::ConstSharedPtr &measurementMsg);
        void HandleSamplerCtl(const SamplerControlMsg::ConstSharedPtr &samplerCtlMsg);
        void HandleMissionCmd(const MissionCmd &missionCmd);
        uint8_t getMissionCmd();
        void retranslateSamplerCtrlMsg(const MissionCmd::SharedPtr &missionCmd);

        void stopExecuting();

        void updateFeedback();
        
        void send_rotor_velocity(double vel);
        bool get_measurements();

        bool drillStuck();

    private:
     

        enum class State{
        IDLE,
        CALIBRATE_PLATFORM,
        CALIBRATE_DRILL,
        CALIBRATE_CONTAINER,

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
        MANUAL_CONTROL,

        STOP,
        ABORT,
        DONE
        };

        enum class RECOVER_STATE{
            LIFT_UP,
            WAIT,
            CHECK_ROTATION
        };

        enum class CONTROL_TYPE{
            NO_SAMPLER,
            MANUAL,
            AUTONOMY
        };

        void setControlType(const RoverStatusMsg::ConstSharedPtr &msg);


        const double MIN_SPEED = 3.0;
        const double MAX_CURRENT = 3.0;
        const double MIN_TIME = 0.5;
        const int MAX_RECOVERY_ATTEMPT = 5;
        bool stall_timer_running_ = false;
        rclcpp::Time stall_start_time_;
        int recovery_attempt = 0;
        RECOVER_STATE recover_state = RECOVER_STATE::LIFT_UP;


        std::string to_string(State s);
        
        uint8_t to_Feedback(State s);

        State state_ = State::IDLE;
        State state_to_abort = State::IDLE;
        State state_to_stop = State::IDLE;
        MissionMsg::SharedPtr missionFeedbackMsg;
        MissionCmd::SharedPtr missionCmdMsg;
        MissionCmd::SharedPtr LastMissionCmdMsg;

        void statesLoop();
        void AppFeedbackPublish();
        int getPlatformPosition();


        std::unique_ptr<JointMovement> joints_;
       
        //rclcpp::Subscription<rex_interfaces::msg::SamplerControl>::SharedPtr SubStatus;

        std_msgs::msg::String::SharedPtr SubStatus;
        rclcpp::Subscription<rex_interfaces::msg::RoverStatus>::SharedPtr mRoverStatus_;
        rclcpp::Publisher<rex_interfaces::msg::SamplerFeedback>::SharedPtr PubFeedback_;
        rclcpp::Subscription<MissionCmd>::SharedPtr mMissionCmd_;
        rclcpp::Subscription<rex_interfaces::msg::SamplerFeedback>::SharedPtr MeasurementFeedback_;
        rclcpp::Subscription<rex_interfaces::msg::SamplerControl>::SharedPtr mSamplerCtrl_;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr rotor_velocity_pub_;

        const std::string node_name = "ros_deep_sampler";

        rex_interfaces::msg::SamplerFeedback mPubFeedback;
        rclcpp::TimerBase::SharedPtr appFeedbackTimer;


        rclcpp::TimerBase::SharedPtr missionTimer_;
        rclcpp::TimerBase::SharedPtr AppFeedbackTimer_;


        std::string mission_commands;
        std::vector<JointMovement::JointCommand> commands;
        //std::vector<sampler_motion_interfaces::msg::ActuatorCommand>  commands;
        bool goal_sent;
        double platform_position = -100.0;
        double drill_position = -100.0;
        double drill_velocity = 0.0;
        double container_a_pos = 0.0;
        int time_between_states = 100;
        const double tolerance = 0.001; // meters
        int rotation_time = 0;
        bool calibrate_drill = false;
        bool calibrate_platform = false;
        bool mission_in_stop = false;
        bool new_mission_cmd = false;

       
        RoverStatusMsg::ConstSharedPtr LastStatusMsg;
        SamplerControlMsg::ConstSharedPtr LastCtrlMsg;
        CONTROL_TYPE ctrlType_ = CONTROL_TYPE::NO_SAMPLER;

        unsigned int CONTROL_MODE_DEEP_SAMPLER = 32;
        unsigned int CONTROL_MODE_SURFACE_SAMPLER = 64;
        unsigned int CONTROL_MODE_DEEP_SAMPLER_AUTONOMY = 512;
        unsigned int CONTROL_MODE_SURFACE_SAMPLER_AUTONOMY = 1024;

        //Required Measurements 
        double weight_a = 0.0;
        double ph = 0.0;


};

}
