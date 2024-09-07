#include <can_wrapper/ProbeStatusForwarder.hpp>

ProbeStatusForwarder::ProbeStatusForwarder(ros::NodeHandle &nh):
    mNh(nh)
{
    mProbeStatusGrabber = nh.subscribe<can_msgs::Frame>(RosCanConstants::RosTopics::can_raw_RX,1024,&ProbeStatusForwarder::probeStatusGrabber,this);
	mProbeStatusPublisher = nh.advertise<can_wrapper::ProbeStatus>(RosCanConstants::RosTopics::can_probe_status,256);
}

void ProbeStatusForwarder::probeStatusGrabber(const can_msgs::Frame::ConstPtr &frame)
{
    auto rf = VescInterop::rosToVesc(*frame);

    switch(rf.command)
    {
        case VESC_COMMAND_STATUS_8:
            VESC_Status_8 status8;
            VESC_ZeroMemory(&status8, sizeof(status8));
            VESC_convertRawToStatus8(&status8, &rf);
			newStatus8(status8);
        	break;
        case VESC_COMMAND_STATUS_9:
            VESC_Status_9 status9;
            VESC_ZeroMemory(&status9, sizeof(status9));
            VESC_convertRawToStatus9(&status9, &rf);
			newStatus9(status9);
        	break;
		default:
			return;
    };

	statusPublisher();
}

void ProbeStatusForwarder::newStatus8(VESC_Status_8 &status8)
{
	mStatus8 = status8;
	mStatus8Fresh = true;
	mStatus8Staleness = 0;

	if(mStatus9Staleness < UINT8_MAX)
		mStatus9Staleness++;
}

void ProbeStatusForwarder::newStatus9(VESC_Status_9 & status9)
{
	mStatus9 = status9;
	mStatus9Fresh = true;
	mStatus9Staleness = 0;

	if(mStatus8Staleness < UINT8_MAX)
		mStatus8Staleness++;
}

void ProbeStatusForwarder::warnRotten(int statusNum)
{
	if(!mRottenNoted)
	{
		mRottenNoted = true;
		ROS_WARN("[ProbeStatusForwarder]: Status %i was not updated recently while other was (Threshold (%i) reached). Publishing of ProbeStatus will resume with last value!", statusNum, ROTTEN_THRESHOLD);
	}
}

void ProbeStatusForwarder::infoNotRotten()
{
	if(mRottenNoted)
	{
		mRottenNoted = false;
		ROS_INFO("[ProbeStatusForwarder]: All statuses were updated recently.");
	}
}

void ProbeStatusForwarder::statusPublisher()
{	
	if(mStatus8Staleness > ROTTEN_THRESHOLD)
	{
		warnRotten(8);
	}
	else if(mStatus9Staleness > ROTTEN_THRESHOLD)
	{
		warnRotten(9);
	}
	else if(!mStatus8Fresh || !mStatus9Fresh)
		return;

	can_wrapper::ProbeStatus status;

	status.weightA = mStatus8.weightA;
	status.distance = mStatus8.distance;
	status.humidity = mStatus8.humidity;
	status.vibrations = mStatus8.vibrations;
	status.weightB = mStatus8.weightB;

	status.potassium = mStatus9.potassium;
	status.nitrogen = mStatus9.nitrogen;
	status.phosphorus = mStatus9.phosphorus;
	status.ph = mStatus9.ph;

	status.header.stamp = ros::Time::now();

	mProbeStatusPublisher.publish(status);

	if(mStatus8Fresh && mStatus9Fresh)
	{
		infoNotRotten();
		mStatus8Fresh = false;
		mStatus9Fresh = false;
	}

}