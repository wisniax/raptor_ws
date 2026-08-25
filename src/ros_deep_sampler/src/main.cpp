#include <cstdio>
#include "rclcpp/rclcpp.hpp"
#include "ros_deep_sampler/mission_control.hpp"




// int main(int argc, char ** argv)
// {
//   rclcpp::init(argc, argv);

//   auto mission = std::make_shared<ros_deep_sampler::MissionControl>(rclcpp::NodeOptions());
//   // auto move_server = std::make_shared<ros_deep_sampler::MoveLinearActionServer>(rclcpp::NodeOptions());
//   // auto hardware_bridge = std::make_shared<ros_deep_sampler::HardwareBridge>(rclcpp::NodeOptions());

//   rclcpp::executors::MultiThreadedExecutor exec;
//   exec.add_node(mission);
//   // exec.add_node(move_server);
//   // exec.add_node(hardware_bridge);
//   // exec.add_node(mission->get_move_client());
//   exec.spin();

//   rclcpp::shutdown();
// }

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    auto mission =
        std::make_shared<ros_deep_sampler::MissionControl>(
            rclcpp::NodeOptions());

    rclcpp::spin(mission);

    rclcpp::shutdown();
}
