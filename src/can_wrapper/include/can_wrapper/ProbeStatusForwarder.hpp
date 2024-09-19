#ifndef ProbeStatusForwarder_h_
#define ProbeStatusForwarder_h_

#include <ros/ros.h>
#include <boost/shared_ptr.hpp>
#include <can_wrapper/ProbeStatus.h>
#include <can_wrapper/RosCanConstants.hpp>
#include <can_wrapper/VescInterop.hpp>
#include <can_msgs/Frame.h>
extern "C"
{
#include <libVescCan/VESC.h>
}

class ProbeStatusForwarder
{
public:
	ProbeStatusForwarder(ros::NodeHandle& nh);

private:
	void probeStatusGrabber(const can_msgs::Frame::ConstPtr &frame);
	void statusPublisher();

	void newStatus8(VESC_Status_8& status8);
	void newStatus9(VESC_Status_9& status9);

	void warnRotten(int statusNum);
	void infoNotRotten();


	const uint8_t ROTTEN_THRESHOLD = 10;

	ros::NodeHandle mNh;

	bool mRottenNoted = false;

	VESC_Status_8 mStatus8;
	bool mStatus8Fresh = false;
	uint8_t mStatus8Staleness = 0;

	VESC_Status_9 mStatus9;
	bool mStatus9Fresh = false;
	uint8_t mStatus9Staleness = 0;

	ros::Subscriber mProbeStatusGrabber;
	ros::Publisher mProbeStatusPublisher;	
};

#endif //ProbeStatusForwarder_h_