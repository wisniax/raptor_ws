#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "rex_interfaces/msg/sampler_control.hpp"
#include "rex_interfaces/msg/sampler_feedback.hpp"
#include "rex_interfaces/msg/rover_status.hpp"
#include "ros_deep_sampler/linear_movement_client.hpp"
#include "ros_deep_sampler/linear_movement_server.hpp"
#include "sampler_motion_interfaces/msg/sampler_can_ex.hpp"
#include <chrono>
#include <string>
#include <iostream>
#include "ros_deep_sampler/RosCanConstants.hpp"


namespace ros_deep_sampler{

using SamplerControlMsg = rex_interfaces::msg::SamplerControl;
using RoverStatusMsg = rex_interfaces::msg::RoverStatus;


class MissionControl : public rclcpp::Node{
    public:
        explicit MissionControl(const rclcpp::NodeOptions & options);
         
        std::shared_ptr<MoveLinearActionClient> get_move_client();

        int check_drilling();

    private:
        void MissionCheck(std_msgs::msg::String::SharedPtr msg);
        


        void publishAppFeedback();

        void startMission();
        
        
        bool isSamplerMode(const RoverStatusMsg::ConstSharedPtr &msg);
        void HandleRoverStatus(const RoverStatusMsg::ConstSharedPtr &roverStatusMsg);
        void HandleSamplerCtl(const SamplerControlMsg::ConstSharedPtr &samplerCtlMsg);
     

        enum class State{
            IDLE,
            CALIBRATE_PLATFORM,
            CALIBRATE_DRILL,
            MOVE_PLATFORM_DOWN,
            DRILLING,
            MOVE_DRILL_UP,
            MOVE_PLATFORM_UP,
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

       
        //rclcpp::Subscription<rex_interfaces::msg::SamplerControl>::SharedPtr SubStatus;
        std::shared_ptr<MoveLinearActionClient> move_client_;
        std_msgs::msg::String::SharedPtr SubStatus;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
        rclcpp::Subscription<rex_interfaces::msg::RoverStatus>::SharedPtr mStatus_;
        rclcpp::Publisher<rex_interfaces::msg::SamplerFeedback>::SharedPtr PubFeedback_;
        rclcpp::Subscription<rex_interfaces::msg::SamplerControl>::SharedPtr mSamplerCtrl_;
        rclcpp::Publisher<sampler_motion_interfaces::msg::SamplerCanEx>::SharedPtr mPubCanCtrl_;

        const std::string node_name = "ros_deep_sampler";

        rex_interfaces::msg::SamplerFeedback mPubFeedback;
        rclcpp::TimerBase::SharedPtr appFeedbackTimer;


        rclcpp::TimerBase::SharedPtr missionTimer_;
        rclcpp::TimerBase::SharedPtr AppFeedbackTimer_;


        std::string mission_commands;
        std::vector<sampler_motion_interfaces::msg::ActuatorCommand>  commands;
        sampler_motion_interfaces::msg::ActuatorCommand cmd1;
        sampler_motion_interfaces::msg::ActuatorCommand cmd3;
        sampler_motion_interfaces::msg::ActuatorCommand cmd2;
        bool goal_sent;
        double platform_position = -100.0;
        double drill_position = -100.0;
        double drill_velocity = 0.0;
        int time_between_states = 100;
        const double tolerance = 0.001; // meters
        bool calibrate_drill = false;
        bool calibrate_platform = false;
        int mission_step = 0; // 0 - begining, 1 - get sample; 2 - put sample; 3 - 
        RoverStatusMsg::ConstSharedPtr LastStatusMsg;
        SamplerControlMsg::ConstSharedPtr LastCtrlMsg;


};

}
