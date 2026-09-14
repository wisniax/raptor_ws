#include "can_bridge/CanHotplug.hpp"

#include <chrono>
#include <stdexcept>
#include <linux/netlink.h>
#include <net/if.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <lifecycle_msgs/msg/transition.hpp>
#include <rclcpp_components/register_node_macro.hpp>

CanHotplug::CanHotplug(const rclcpp::NodeOptions &options) : Node("can_hotplug", options)
{
    std::string socketcan_namespace = this->declare_parameter<std::string>("socketcan_namespace", "");
    if (!socketcan_namespace.empty() &&
        socketcan_namespace.back() != '/')
    {
        socketcan_namespace += '/';
    }

    sender_lifecycle_client_ = create_client<lifecycle_msgs::srv::ChangeState>(
        socketcan_namespace + "socket_can_sender/change_state");
    receiver_lifecycle_client_ = create_client<lifecycle_msgs::srv::ChangeState>(
        socketcan_namespace + "socket_can_receiver/change_state");

    sender_lifecycle_client_->wait_for_service(std::chrono::seconds(5));
    receiver_lifecycle_client_->wait_for_service(std::chrono::seconds(5));

    RCLCPP_DEBUG(
        get_logger(),
        "Sender lifecycle service: %s",
        sender_lifecycle_client_->get_service_name());

    RCLCPP_DEBUG(
        get_logger(),
        "Receiver lifecycle service: %s",
        receiver_lifecycle_client_->get_service_name());

    using Transition = lifecycle_msgs::msg::Transition;
    restart_sequence_ = {
        {sender_lifecycle_client_, Transition::TRANSITION_DEACTIVATE},
        {receiver_lifecycle_client_, Transition::TRANSITION_DEACTIVATE},

        {sender_lifecycle_client_, Transition::TRANSITION_CLEANUP},
        {receiver_lifecycle_client_, Transition::TRANSITION_CLEANUP},

        {receiver_lifecycle_client_, Transition::TRANSITION_CONFIGURE},
        {sender_lifecycle_client_, Transition::TRANSITION_CONFIGURE},

        // Auto-active is on by default, so this is not needed
        // {receiver_lifecycle_client_, Transition::TRANSITION_ACTIVATE},
        // {sender_lifecycle_client_, Transition::TRANSITION_ACTIVATE},
    };

    interface_name_ = this->declare_parameter<std::string>("interface", "can0");

    setup_netlink();

    if (if_nametoindex(interface_name_.c_str()) == 0)
    {
        // If the CAN interface doesn't exist at startup, mark it as destroyed to allow later recovery
        interface_destroyed_ = true;

        RCLCPP_INFO(this->get_logger(), "CAN interface %s doesn't exist at startup", interface_name_.c_str());
    }

    watcher_thread_ = std::thread(&CanHotplug::watch_loop, this);
}

CanHotplug::~CanHotplug()
{
    stop_ = true;

    if (watcher_thread_.joinable())
        watcher_thread_.join();

    if (netlink_fd_ >= 0)
        close(netlink_fd_);
}

void CanHotplug::setup_netlink()
{
    netlink_fd_ = socket(AF_NETLINK, SOCK_RAW, NETLINK_ROUTE);

    if (netlink_fd_ < 0)
        throw std::runtime_error("Couldn't create netlink socket");

    sockaddr_nl addr{};
    addr.nl_family = AF_NETLINK;
    addr.nl_groups = RTMGRP_LINK;

    if (bind(netlink_fd_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0)
    {
        close(netlink_fd_);
        netlink_fd_ = -1;
        throw std::runtime_error("Couldn't bind to netlink socket");
    }
}

void CanHotplug::watch_loop()
{
    char buffer[8192];

    while (!stop_)
    {
        pollfd pfd{};
        pfd.fd = netlink_fd_;
        pfd.events = POLLIN;
        pfd.revents = 0;

        // Check if data ready
        if (poll(&pfd, 1, 500) <= 0)
            continue;

        // Read data
        const auto size = recv(netlink_fd_, buffer, sizeof(buffer), 0);

        if (size <= 0)
            continue;

        int remaining = static_cast<int>(size);

        // Iterate over rtnetlink messages
        for (
            auto *netlink_header = reinterpret_cast<nlmsghdr *>(buffer);
            NLMSG_OK(netlink_header, remaining);
            netlink_header = NLMSG_NEXT(netlink_header, remaining))
        {
            // Handle netlink message

            // Only listen for new-link and del-link messages
            if (netlink_header->nlmsg_type != RTM_NEWLINK && netlink_header->nlmsg_type != RTM_DELLINK)
            {
                continue;
            }

            auto *interface_info = reinterpret_cast<ifinfomsg *>(NLMSG_DATA(netlink_header));

            char *name = nullptr;

            // Iterate over route attributes to find interface name
            int len = IFLA_PAYLOAD(netlink_header);
            for (
                auto *attr = IFLA_RTA(interface_info);
                RTA_OK(attr, len);
                attr = RTA_NEXT(attr, len))
            {
                if (attr->rta_type == IFLA_IFNAME)
                    name = static_cast<char *>(RTA_DATA(attr));
            }

            if (!name || interface_name_ != name)
                continue;

            if (netlink_header->nlmsg_type == RTM_NEWLINK)
                handle_newlink(interface_info);
            else
                handle_dellink();
        }
    }
}

void CanHotplug::handle_newlink(const ifinfomsg *interface_info)
{
    if (!interface_destroyed_)
        return;

    // Wait for interface UP
    if (!(interface_info->ifi_flags & IFF_UP))
        return;

    if (restarting_.exchange(true))
        return;

    interface_destroyed_ = false;

    RCLCPP_INFO(this->get_logger(), "CAN interface created, restarting SocketCAN node");

    run_next_step(0);
}

void CanHotplug::handle_dellink()
{
    interface_destroyed_ = true;
    RCLCPP_INFO(this->get_logger(), "CAN interface destroyed, ready to restart SocketCAN node");
}

void CanHotplug::run_next_step(size_t step_index)
{
    if (step_index >= restart_sequence_.size())
    {
        restarting_ = false;
        RCLCPP_INFO(this->get_logger(), "Finished restarting");
        return;
    }

    auto step = restart_sequence_[step_index];
    auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
    request->transition.id = step.transition;
    RCLCPP_DEBUG(this->get_logger(), "Running step %d", step_index);

    step.client->async_send_request(
        request,
        [this, step_index](rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedFuture future)
        {
            RCLCPP_DEBUG(
                this->get_logger(),
                "Response for step %d, success=%d",
                step_index,
                future.get()->success);
            run_next_step(step_index + 1);
        });
}

RCLCPP_COMPONENTS_REGISTER_NODE(CanHotplug)
