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

/**
 * @brief Restarts SocketCAN bridge nodes when a CAN interface is recreated (typically due to adapter unplug & replug)
 *
 * Listens for Linux rtnetlink notifications for events regarding the specified interface.
 * If an interface destruction followed by a re-creation is detected, SocketCAN nodes are restarted
 * via the lifecycle mechanism.
 * Also handles the case of the interface being absent at node startup.
 *
 * @par ROS parameters
 * - `interface` (string, default: `can0`): the CAN interface to monitor
 * - `socketcan_namespace` (string, default: empty): allows a different ROS namespace to be specified for the SocketCAN nodes
 *   by default inherits this node's namespace.
 */
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

    /**
     * @brief Setup the socket to listen for rtnelink interface updates.
     * @throws std::runtime_error If cannot create or bind to socket
     */
    void setup_netlink();

    /**
     * @brief Main watch loop. Listens to Linux kernel rtnetlink for interface events.
     */
    void watch_loop();

    /**
     * @brief Handles interface creation/update event sent by rtnetlink.
     *
     * Only triggers the restart sequence if interface had been destroyed/missing
     * and the interface is being brought up.
     *
     * @param interface_info Interface info from rtnetlink
     */
    void handle_newlink(const ifinfomsg *interface_info);

    /**
     * @brief Handles the interface destruction event sent by rtnetlink
     */
    void handle_dellink();

    /**
     * @brief Performs one asynchronous step of the restart sequence and enqueues the next one.
     * @param step_index Index of the step in the restart sequence.
     */
    void run_next_step(size_t step_index);

    std::string interface_name_;

    int netlink_fd_{-1};
    std::thread watcher_thread_;

    bool interface_destroyed_{false};

    std::atomic<bool> restarting_{false};
    std::atomic<bool> stop_{false};
};

#endif // CanHotplug_h_
