#ifndef VescStatusHandler_h_
#define VescStatusHandler_h_

#include <ros/ros.h>
#include <unordered_map>
#include <boost/algorithm/algorithm.hpp>
#include <boost/shared_ptr.hpp>
#include <can_wrapper/VescStatus.h>
#include <can_wrapper/RosCanConstants.hpp>
#include <can_wrapper/VescInterop.hpp>
#include <can_msgs/Frame.h>
extern "C"
{
#include <libVescCan/VESC.h>
}

struct MotorStatusKey
{
	VESC_Id_t vescId;
	VESC_Command commandId;
	MotorStatusKey() = default;
	inline MotorStatusKey(VESC_Id_t vescId, VESC_Command commandId) :
		vescId(vescId), commandId(commandId)
	{}
	
	inline bool operator==(const MotorStatusKey &rhs) const
	{
		return vescId == rhs.vescId && commandId == rhs.commandId;
	}
};

struct MotorStatusValue
{
	VESC_RawFrame vescFrame;
	ros::Time recivedTime;
	MotorStatusValue() = default;
	inline MotorStatusValue(VESC_RawFrame vescFrame, ros::Time recivedTime) :
		vescFrame(vescFrame), recivedTime(recivedTime)
	{}
};

template<class T>
struct MyHash;

template<>
struct MyHash<MotorStatusKey>
{
public:
	inline std::size_t operator()(MotorStatusKey const& key) const 
	{
		std::size_t h1 = std::hash<uint8_t>()(key.vescId);
		std::size_t h2 = std::hash<uint8_t>()(uint8_t(key.commandId));
		return h1 ^ (h2 << 1);
	}
};

class VescStatusHandler
{
public:
	VescStatusHandler(ros::NodeHandle& nh);

	void statusGrabber(const can_msgs::Frame::ConstPtr &frame);
	void sendUpdate(uint8_t vescId);
	void clear();

private:

	ros::Time lastSendTime;

	std::unordered_map<MotorStatusKey, MotorStatusValue, MyHash<MotorStatusKey>> mMotorStatus;

	ros::Subscriber mStatusGrabber;
	ros::Publisher mStatusPublisher;

	ros::Timer mMotorCommandTimer;
	
};



#endif //VescMotorController_h_