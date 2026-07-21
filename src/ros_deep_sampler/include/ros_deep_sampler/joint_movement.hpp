#include <memory>
#include "sampler_motion_interfaces/action/move_linear.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "ros_deep_sampler/visibility_control.h"

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




namespace ros_deep_sampler{

class  JointMovement{
    public:
        
        explicit JointMovement(rclcpp::Node *node);

        void movePlatform(double pos, double vel, bool calibrated);
        void moveDrill(double pos, double vel, bool calibrated);
        void moveContainer(double pos, double vel, bool calibrated);

        void send_rotor_velocity(double vel);

        
        void generateTrajectory(
            trajectory_msgs::msg::JointTrajectory &traj,
            const std::string &joint_name,
            double start_pos,
            double final_pos,
            double max_velocity,
            bool is_calibration,
            double accel_time = 0.5,
            double dt = 0.02);

        //ACTION_TUTORIALS_CPP_PUBLIC
        
        double get_current_position(int id);
        double get_current_velocity(int id);

        bool isMoving() const;

        bool goalReached() const;

        bool goalFailed() const;

        void cancel_movement();
        void stop_movement();

        void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);

        using FollowJointTrajectory =
            control_msgs::action::FollowJointTrajectory;

        using TJCGoalHandle =
            rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;

    private:
        rclcpp::Node::SharedPtr node_;
        //rclcpp_action::Client<FollowJointTrajectory>::SharedPtr tjc_client_;
        rclcpp_action::Client<FollowJointTrajectory>::SharedPtr platform_client_;
        rclcpp_action::Client<FollowJointTrajectory>::SharedPtr drill_client_;
        rclcpp_action::Client<FollowJointTrajectory>::SharedPtr container_client_;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr rotor_velocity_pub_;

        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;

        TJCGoalHandle::SharedPtr active_tjc_goal_;


        std::unordered_map<std::string, double> current_position_;
        std::unordered_map<std::string, double> current_velocity_;

        double prev_platform_pos = 0.0;
        double prev_drill_pos = 0.0;
};



}