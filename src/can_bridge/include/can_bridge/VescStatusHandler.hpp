#ifndef VescStatusHandler_h_
#define VescStatusHandler_h_

#include "rclcpp/rclcpp.hpp"
#include <unordered_map>
#include <boost/algorithm/algorithm.hpp>
#include <boost/shared_ptr.hpp>
#include <rex_interfaces/msg/vesc_status.hpp>
#include <can_bridge/RosCanConstants.hpp>
#include <can_bridge/VescInterop.hpp>
#include <can_msgs/msg/frame.hpp>
extern "C"
{
#include <libVescCan/VESC.h>
}

struct MotorStatusKey
{
	VESC_Id_t vescId;
	VESC_Command commandId;
	MotorStatusKey() = default;
	inline MotorStatusKey(VESC_Id_t vescId, VESC_Command commandId) : vescId(vescId), commandId(commandId)
	{
	}

	inline bool operator==(const MotorStatusKey &rhs) const
	{
		return vescId == rhs.vescId && commandId == rhs.commandId;
	}
};

struct MotorStatusValue
{
	VESC_RawFrame vescFrame;
	rclcpp::Time receivedTime;
	MotorStatusValue() = default;
	inline MotorStatusValue(VESC_RawFrame vescFrame, rclcpp::Time recivedFrameTime) : vescFrame(vescFrame), receivedTime(recivedFrameTime)
	{
	}
};

template <class T>
struct MyHash;

template <>
struct MyHash<MotorStatusKey>
{
public:
	inline std::size_t operator()(MotorStatusKey const &key) const
	{
		std::size_t h1 = std::hash<uint8_t>()(key.vescId);
		std::size_t h2 = std::hash<uint8_t>()(uint8_t(key.commandId));
		return h1 ^ (h2 << 1);
	}
};

class VescStatusHandler
{
public:
	VescStatusHandler(rclcpp::Node::SharedPtr &nh);

	void statusGrabber(const can_msgs::msg::Frame::ConstSharedPtr &frame);
	void sendUpdate(uint8_t vescId);
	void clear();

private:
	rclcpp::Time lastSendTime;

	std::unordered_map<MotorStatusKey, MotorStatusValue, MyHash<MotorStatusKey>> mMotorStatus;

	rclcpp::Node::SharedPtr mNh;
	rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr mStatusGrabber;
	rclcpp::Publisher<rex_interfaces::msg::VescStatus>::SharedPtr mStatusPublisher;
	rclcpp::TimerBase::SharedPtr mMotorCommandTimer;
};

#endif // VescMotorController_h_