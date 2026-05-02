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
            MOVE_UP_CALIBRATION,
            MOVE_PLATFORM_DOWN,
            DRILLING,
            MOVE_DRILL_UP,
            MOVE_PLATFORM_UP,
            DONE,
            ABORT

        };

        
        static const uint8_t sampler_platform_id = 0x80;		 /**< ID for the sampler platform movement */
        static const uint8_t sampler_drill_mov_id = 0x81;		 /**< ID for the sampler drill movement */
        static const uint8_t sampler_drill_id = 0x82;			 /**< ID for the sampler drill action */
        static const uint8_t sampler_container_a_id = 0x83;	 /**< ID for the sampler container A (position only) */
        static const uint8_t sampler_container_b_id = 0x84;	 /**< ID for the sampler container B (position only) */
        static const uint8_t sampler_vacuum_suction_id = 0x85; /**< ID for the sampler vacuum main motor -1:1 duty range */
        static const uint8_t sampler_vacuum_a_id = 0x86;		 /**< ID for the sampler vacuum motor A ON-OFF only (duty 0-1) */
        static const uint8_t sampler_vacuum_b_id = 0x87;		 /**< ID for the sampler vacuum motor B ON-OFF only (duty 0-1) */
        

        

        std::string to_string(State s)
        {
            switch(s) {
                case State::IDLE: return "IDLE";
                case State::MOVE_UP_CALIBRATION: return "MOVING up to set origin";
                case State::DRILLING: return "DRILLING";
                case State::DONE: return "DONE";
                default: return "UNKNOWN";
            }
        }

        State state_ = State::IDLE;
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
        float platform_position = 0.0;
        float drill_position = 0.0;
        float drill_velocity = 0.0;
        int time_between_states = 100;
        RoverStatusMsg::ConstSharedPtr LastStatusMsg;
        SamplerControlMsg::ConstSharedPtr LastCtrlMsg;


};

}
