#include "ros_deep_sampler/joint_movement.hpp"


namespace ros_deep_sampler{

JointMovement::JointMovement(rclcpp::Node *node)
: node_(node)
{
    platform_client_ =
        rclcpp_action::create_client<FollowJointTrajectory>(
            node_,
            "/platform_controller/follow_joint_trajectory");

    drill_client_ =
        rclcpp_action::create_client<FollowJointTrajectory>(
            node_,
            "/drill_position_controller/follow_joint_trajectory");

    container_client_ =
        rclcpp_action::create_client<FollowJointTrajectory>(
            node_,
            "/container_controller/follow_joint_trajectory");

    rotor_velocity_pub_ =
        node_->create_publisher<std_msgs::msg::Float64>(
            "/rotor_joint_cmd", 10);

    // joint_sub_ =
    //     node_->create_subscription<sensor_msgs::msg::JointState>(
    //         "/joint_states",
    //         10,
    //         std::bind(
    //             &JointMovement::jointStateCallback,
    //             this,
    //             std::placeholders::_1));
}




}
