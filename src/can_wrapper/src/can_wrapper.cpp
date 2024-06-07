#include <ros/ros.h>
#include <memory>
#include <linux/can.h>
#include <can_msgs/Frame.h>
#include <std_msgs/String.h>

#include "CM/CM.h"
#include "can_wrapper/MotorVelocityFeedback.hpp"
#include "can_wrapper/MotorControl.hpp"
#include "can_wrapper/RosCanConstants.hpp"
#include "can_wrapper/VescMotorController.hpp"
#include "can_wrapper/RoverControl.h"

#include <ros/service.h>
#include <std_srvs/SetBool.h>

#define RPM_MAX 500

enum class CanNodeMode
{
	Created,
	Opening,
	Opened,
	Closing,
	Closed,
	Faulted
};

double XVelAxis;
double ZRotAxis;

static void roverControlCallback(const can_wrapper::RoverControl::ConstPtr &msg)
{
	XVelAxis = msg->XVelAxis;
	ZRotAxis = msg->ZRotAxis;

	// Process the rover control message here
}

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "can_wrapper");
	ros::NodeHandle n;

	MotorControl motorControl(n);
	VescMotorController vmc;

	CanNodeMode canNodeMode = CanNodeMode::Created;
	ros::Rate rate(1000);

	ros::Subscriber sub = n.subscribe("/MQTT/RoverControl", 100, roverControlCallback);

	std_srvs::SetBool::Request req;
	std_srvs::SetBool::Response res;
	can_wrapper::Wheels vel;

	bool forward;
	float left;
	float right;

	while (ros::ok())
	{
		can_wrapper::Wheels vel;
		switch (canNodeMode)
		{
		case CanNodeMode::Created:

			if (!ros::service::waitForService("/CAN/ros_can_integration/check_status", 15))
			{
				ROS_WARN("Service /CAN/ros_can_integration/check_status is not yet available. Retrying...");
				canNodeMode = CanNodeMode::Faulted;
				break;
			}

			if (!ros::service::call("/CAN/ros_can_integration/check_status", req, res))
			{
				ROS_WARN("Service /CAN/ros_can_integration/check_status call failed. Retrying...");
				canNodeMode = CanNodeMode::Faulted;
				break;
			}

			if (res.success != 0)
			{
				ROS_WARN("Package ros_can_integration is not yet ready... Retrying...");
				canNodeMode = CanNodeMode::Faulted;
				break;
			}
			canNodeMode = CanNodeMode::Opening;
			break;

		case CanNodeMode::Opening:
			ros::Duration(0.1).sleep();

			canNodeMode = CanNodeMode::Opened;

			break;

		case CanNodeMode::Opened:
			forward = XVelAxis > 0.0f;
			left = (std::abs(XVelAxis) + (std::max(ZRotAxis,0.0) * -2.0)) * RPM_MAX;
			right = (std::abs(XVelAxis) + (std::min(ZRotAxis,0.0) * 2.0)) * RPM_MAX;

			if(!forward)
			{
				left = -left;
				right = -right;
			}

			//ROS_INFO("Left: %01.2f ; Right: %01.2f", left, right);

			vel.header.stamp = ros::Time::now();

			vel.frontLeft.commandIdAngle = 0; //nope
			vel.frontLeft.setAngle = 0;	
			vel.frontRight.commandIdAngle = 0; //nope again
			vel.frontRight.setAngle = 0;	
			vel.rearLeft.commandIdAngle = 0; //still nope
			vel.rearLeft.setAngle = 0;	
			vel.rearRight.commandIdAngle = 0; //nope nope nope
			vel.rearRight.setAngle = 0;

			vel.frontLeft.commandId = (int)VescCan::Consts::Command::SET_RPM;
			vel.frontLeft.setValue = left;
			vel.rearLeft.commandId = (int)VescCan::Consts::Command::SET_RPM;
			vel.rearLeft.setValue = left;

			vel.frontRight.commandId = (int)VescCan::Consts::Command::SET_RPM;
			vel.frontRight.setValue = right;
			vel.rearRight.commandId = (int)VescCan::Consts::Command::SET_RPM;
			vel.rearRight.setValue = right;
			
			motorControl.sendMotorVel(vel);
			break;

		case CanNodeMode::Closing:
			ROS_INFO("CanNodeMode::Closing");

			canNodeMode = CanNodeMode::Closed;
			break;
		case CanNodeMode::Closed:

			canNodeMode = CanNodeMode::Opening;
			break;
		case CanNodeMode::Faulted:
			ros::Duration(5).sleep();
			canNodeMode = CanNodeMode::Created;
			break;
		default:
			ROS_INFO("CanNodeMode::Unknown");
			canNodeMode = CanNodeMode::Faulted;
			break;
		}
		rate.sleep();
		ros::spinOnce();
	}
}