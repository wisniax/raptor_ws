#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "rex_interfaces/msg/sampler_control.hpp"
#include "rex_interfaces/msg/sampler_feedback.hpp"
#include "rex_interfaces/msg/rover_status.hpp"
#include <chrono>
#include <string>
#include <iostream>
#include "ros_deep_sampler/RosCanConstants.hpp"
#include "ros_deep_sampler/joint_movement.hpp"


namespace ros_deep_sampler{

using SamplerControlMsg = rex_interfaces::msg::SamplerControl;
using RoverStatusMsg = rex_interfaces::msg::RoverStatus;
using MeasurementMsg = rex_interfaces::msg::SamplerFeedback;


class MissionControl : public rclcpp::Node{
    public:
        explicit MissionControl(const rclcpp::NodeOptions & options);
         

        int check_drilling();

    private:
        void MissionCheck(std_msgs::msg::String::SharedPtr msg);
        


        void publishAppFeedback();

        void startMission();
        
        
        bool isSamplerMode(const RoverStatusMsg::ConstSharedPtr &msg);
        void HandleRoverStatus(const RoverStatusMsg::ConstSharedPtr &roverStatusMsg);
        void HandleMeasurementFeedback(const MeasurementMsg::ConstSharedPtr &measurementMsg);
        void HandleSamplerCtl(const SamplerControlMsg::ConstSharedPtr &samplerCtlMsg);
        
        void send_rotor_velocity(double vel);
        bool get_measurements();
     

        enum class State{
            IDLE,
            CALIBRATE_PLATFORM,
            CALIBRATE_DRILL,
            MOVE_PLATFORM_DOWN,
            DRILLING,
            MOVE_DRILL_UP,
            MOVE_PLATFORM_UP,
            MEASURE_SAMPLE,
            DONE,
            ABORT

        };
        

        std::string to_string(State s)
        {
            switch(s) {
                case State::IDLE: return "IDLE";
                case State::CALIBRATE_PLATFORM: return "MOVING up to set origin";
                case State::MOVE_PLATFORM_DOWN: return "MOVING PLATFORM DOWN";
                case State::DRILLING: return "DRILLING WITH MOVING DRILL DOWN";
                case State::MOVE_DRILL_UP: return "MOVING DRILL UP";
                case State::MOVE_PLATFORM_UP: return "MOVING PLATFORM UP";
                case State::DONE: return "DONE";
                default: return "UNKNOWN";
            }
        }

        State state_ = State::IDLE;
        State state_to_abort = State::IDLE;
        void statesLoop();
        void AppFeedbackPublish();
        int getPlatformPosition();


        std::unique_ptr<JointMovement> joints_;
       
        //rclcpp::Subscription<rex_interfaces::msg::SamplerControl>::SharedPtr SubStatus;

        std_msgs::msg::String::SharedPtr SubStatus;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
        rclcpp::Subscription<rex_interfaces::msg::RoverStatus>::SharedPtr mStatus_;
        rclcpp::Publisher<rex_interfaces::msg::SamplerFeedback>::SharedPtr PubFeedback_;
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
        int time_between_states = 100;
        const double tolerance = 0.001; // meters
        int rotation_time = 0;
        bool calibrate_drill = false;
        bool calibrate_platform = false;
        int measurement_step = 0; // 0 - move container, 1 - put sample; 2 - measure; 3 - move container back
        RoverStatusMsg::ConstSharedPtr LastStatusMsg;
        SamplerControlMsg::ConstSharedPtr LastCtrlMsg;

        //Required Measurements 
        double weight_a = 0.0;
        double ph = 0.0;


};

}
