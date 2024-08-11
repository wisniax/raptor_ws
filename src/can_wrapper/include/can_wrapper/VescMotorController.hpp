#ifndef VescMotorController_h_
#define VescMotorController_h_

#include <ros/ros.h>
#include <unordered_map>
#include <boost/algorithm/algorithm.hpp>
#include <boost/shared_ptr.hpp>
#include <can_wrapper/VescStatus.h>
#include <can_wrapper/RosCanConstants.hpp>
#include <can_wrapper/VescCan/CanFrame.hpp>
#include <can_wrapper/VescCan/CanFrameFactory.hpp>
#include <can_wrapper/VescCan/Converter.hpp>

struct MotorStatusKey
{
	uint8_t vescId;
	VescCan::Consts::Command commandId;
	MotorStatusKey() = default;
	inline MotorStatusKey(uint8_t vescId, VescCan::Consts::Command commandId) :
		vescId(vescId), commandId(commandId)
	{}
	
	inline bool operator==(const MotorStatusKey &rhs) const
	{
		return vescId == rhs.vescId && commandId == rhs.commandId;
	}
};

struct MotorStatusValue
{
	VescCan::CanFrame vescFrame;
	ros::Time recivedTime;
	MotorStatusValue() = default;
	inline MotorStatusValue(VescCan::CanFrame vescFrame, ros::Time recivedTime) :
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

class VescMotorController
{
public:
	VescMotorController(ros::NodeHandle& nh);

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