#include <memory>
#include "rclcpp/rclcpp.hpp"

#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <unordered_map>
#include <string>
#include "std_msgs/msg/float64_multi_array.hpp"
#include "ros_deep_sampler/RosCanConstants.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/float64.hpp"
#include "sampler_motion_interfaces/msg/sampler_mission.hpp"



namespace ros_deep_sampler{
using MissionMsg = sampler_motion_interfaces::msg::SamplerMission;

class  JointMovement{
    public:
        
        explicit JointMovement(rclcpp::Node *node);

        enum class JointsIds{
            PLATFORM,
            DRILL,
            CONTAINER,
            DRILL_ROTOR,
            VACUUM_ROTOR,
            BRUSH_ROTOR,
            FLAP_JOINT
        };

         struct JointCommand
        {
            JointsIds id;
            double position;
            double max_velocity;
            bool calibration = false;
        };

        void movePlatform(double pos, double vel);
        void calibratePlatform(double vel);
        void moveDrill(double pos, double vel, bool calibrated);
        void calibrateDrill(double vel);
        void moveContainer(double pos, double vel, bool calibrated);
        void calibrateContainer(double vel);

        void moveJoints(const std::vector<JointCommand>& commands);
        std::string getJointName(JointsIds id);

        void send_rotor_velocity(JointsIds id, double vel);

        void sendTrajectory(trajectory_msgs::msg::JointTrajectory &traj, JointsIds current_slider);
        void generateTrajectory(
            trajectory_msgs::msg::JointTrajectory &traj,
            const std::string &joint_name,
            double start_pos,
            double final_pos,
            double max_velocity,
            bool is_calibration,
            double accel_time = 0.5,
            double dt = 0.02);
        void setTrajectoryStatus(bool status);

        //ACTION_TUTORIALS_CPP_PUBLIC
        void JointStateFeedback(MissionMsg::SharedPtr &feedbackMsg);
        double get_current_position(JointsIds id);
        double get_current_velocity(JointsIds id);

        // bool isMoving() const;
        bool isTrajectoryFinished();
        //bool goalReached() const;

        bool goalFailed() const;

        void cancelMovement();
        bool isGoalCanceled();
        void setGoalStatus(bool status);
        void stop_movement();
      
        void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);

        using FollowJointTrajectory =
            control_msgs::action::FollowJointTrajectory;

        using TJCGoalHandle =
            rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;

    private:
        rclcpp::Node *node_;
        //rclcpp_action::Client<FollowJointTrajectory>::SharedPtr tjc_client_;
        rclcpp_action::Client<FollowJointTrajectory>::SharedPtr platform_client_;
        rclcpp_action::Client<FollowJointTrajectory>::SharedPtr drill_client_;
        rclcpp_action::Client<FollowJointTrajectory>::SharedPtr container_client_;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr rotor_velocity_pub_;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr vacuum_rotor_velocity_pub_;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr brush_rotor_velocity_pub_;

        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
        
        rclcpp_action::Client<FollowJointTrajectory>::SharedPtr active_client_;
        TJCGoalHandle::SharedPtr active_tjc_goal_;
        std::shared_future<TJCGoalHandle::WrappedResult> result_future_;

        std::unordered_map<std::string, double> current_position_;
        std::unordered_map<std::string, double> current_velocity_;
        JointsIds current_slider_  = JointsIds::PLATFORM; 
        uint8_t goal_state = MissionMsg::GOAL_IDLE;
        uint8_t prev_goal_state = MissionMsg::GOAL_IDLE;

        bool trajectory_finished_;
        bool goal_canceled = false;

        double prev_platform_pos = 0.0;
        double prev_drill_pos = 0.0;




        
};



}