#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "rex_interfaces/msg/sampler_control.hpp"
#include "rex_interfaces/msg/sampler_feedback.hpp"
#include "ros_deep_sampler/linear_movement_client.hpp"
#include "ros_deep_sampler/linear_movement_server.hpp"
#include <chrono>
#include <string>
#include <iostream>


namespace ros_deep_sampler{

class MissionControl : public rclcpp::Node{
    public:
        explicit MissionControl(const rclcpp::NodeOptions & options);
         
        std::shared_ptr<MoveLinearActionClient> get_move_client();

    private:
        void MissionCheck(std_msgs::msg::String::SharedPtr msg);


        void publishAppFeedback();

        void startMission();

        

        enum class State{
            IDLE,
            MOVE_UP_CALIBRATION,
            MOVE_PLATFORM_DOWN,
            DRILLING,
            MOVE_PLATFORM_UP,
            DONE

        };

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
        rclcpp::Publisher<rex_interfaces::msg::SamplerFeedback>::SharedPtr PubFeedback;

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


};

}
