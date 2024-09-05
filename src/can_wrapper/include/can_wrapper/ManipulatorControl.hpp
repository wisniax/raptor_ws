#ifndef ManipulatorControl_h_
#define ManipulatorControl_h_

#include <ros/ros.h>
#include <string>
#include <can_msgs/Frame.h>
#include <array>
#include <can_wrapper/VescInterop.hpp>
#include "can_wrapper/RosCanConstants.hpp"
#include "can_wrapper/ManipulatorControl.h"
extern "C"
{
#include <libVescCan/VESC.h>
}

class ManipulatorControl
{
public:
	ManipulatorControl(const ros::NodeHandle &nh);

private:
	void handleManipulatorCtl(const can_wrapper::ManipulatorControl::ConstPtr& manipulatroCtlMsg);

	can_msgs::Frame encodeStepper(const can_wrapper::Stepper& stepper, const VESC_Id_t vescId);


	ros::NodeHandle mNh;
	ros::Publisher mRawCanPub;		 /**< ROS publisher for raw CAN messages. */
	ros::Subscriber mManipulatorCtlSub; /**< ROS subscriber for motor velocity messages. */
};


#endif //ManipulatorControl_h_