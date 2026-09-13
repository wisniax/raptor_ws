#ifndef CanHotplug_h_
#define CanHotplug_h_

#include <atomic>
#include <cstdint>
#include <string>
#include <thread>
#include <vector>

#include <linux/rtnetlink.h>

#include <lifecycle_msgs/srv/change_state.hpp>
#include <rclcpp/rclcpp.hpp>

class CanHotplug : public rclcpp::Node
{
public:
    CanHotplug(const rclcpp::NodeOptions &options);
    ~CanHotplug() override;

private:
    rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr sender_lifecycle_client_;
    rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr receiver_lifecycle_client_;

    struct RestartStep
    {
        rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr client;
        uint8_t transition;
    };
    std::vector<RestartStep> restart_sequence_;

    void setup_netlink();
    void watch_loop();

    void handle_newlink(const ifinfomsg *interface_info);
    void handle_dellink();

    void run_next_step(size_t step);

    std::string interface_name_;

    int netlink_fd_{-1};
    std::thread watcher_thread_;

    bool interface_destroyed_{false};

    std::atomic<bool> restarting_{false};
    std::atomic<bool> stop_{false};
};

#endif // CanHotplug_h_
