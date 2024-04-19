#ifndef CM_Functions_h_
#define CM_Functions_h_

#if defined(ROS_VERSION_MAJOR) && defined(__cplusplus) && !defined(CM_DISABLE_MSGCONVERT)
#define CM_ENABLE_MSGCONVERT
#endif

#ifdef CM_ENABLE_MSGCONVERT
#include <can_msgs/Frame.h>
#include <ros/time.h>
#endif

#include <math.h>
#include "CM_Message.h"
#include "CM_Consts.h"

/****************************************************************

	convert help

*****************************************************************/

/****************************************************************
		CM_Vector3f for _M functions (whole Message processing)
*****************************************************************/

typedef struct
{
	float x;
	float y;
	float z;
} CM_Vector3f;

/****************************************************************
		convert SetMotorVel
*****************************************************************/

#define CM_MOTORVEL_VALUE_MAX (powf(2.0f, (float)CM_MotorVel_Value_bs) - 1)

inline float CM_convert_SetMotorVel_MaxRpm(CM_StmInit_Var_t rpm_scale)
{
	return (CM_MOTORVEL_VALUE_MAX / rpm_scale);
}

//MotorVel2Rpm

inline float CM_convert_SetMotorVel_mtor(CM_StmInit_Var_t rpm_scale, CM_MotorVel_Value_t motor_val, CM_MotorVel_Dir_t motor_dir)
{
	return (motor_dir == 0 ? motor_val/rpm_scale : -motor_val/rpm_scale);
}

inline void CM_convert_SetMotorVel_mtor_M(CM_Vector3f *rpm_out, const CM_StmInit_Var_t rpm_scale[], const CM_SetMotorVel_msg *msg_in)
{
	rpm_out->x = CM_convert_SetMotorVel_mtor(rpm_scale[0], msg_in->motor_A_value, msg_in->motor_A_dir);
	rpm_out->y = CM_convert_SetMotorVel_mtor(rpm_scale[1], msg_in->motor_B_value, msg_in->motor_B_dir);
	rpm_out->z = CM_convert_SetMotorVel_mtor(rpm_scale[2], msg_in->motor_C_value, msg_in->motor_C_dir);
}

//Rpm2MotorVel

inline void CM_convert_SetMotorVel_rtom(CM_StmInit_Var_t rpm_scale, CM_MotorVel_Value_t *motor_val, CM_MotorVel_Dir_t *motor_dir, float rpm)
{
	*motor_dir = (rpm < 0.0f);
	*motor_val = (CM_MotorVel_Value_t)(*motor_dir ? -rpm * rpm_scale : rpm * rpm_scale);
}

inline void CM_convert_SetMotorVel_rtom_M(CM_GetMotorVel_msg *msg_out, const CM_StmInit_Var_t rpm_scale[], const CM_Vector3f *rpm_in)
{
	msg_out->motor_A_dir = (rpm_in->x < 0.0f);
	msg_out->motor_B_dir = (rpm_in->y < 0.0f);
	msg_out->motor_C_dir = (rpm_in->z < 0.0f);

	msg_out->motor_A_value = (CM_MotorVel_Value_t)( (msg_out->motor_A_dir ? -rpm_in->x : rpm_in->x) * rpm_scale[0]);
	msg_out->motor_B_value = (CM_MotorVel_Value_t)( (msg_out->motor_B_dir ? -rpm_in->y : rpm_in->y) * rpm_scale[1]);
	msg_out->motor_C_value = (CM_MotorVel_Value_t)( (msg_out->motor_C_dir ? -rpm_in->z : rpm_in->z) * rpm_scale[2]);
}

/****************************************************************
		convert GetMotorVel
*****************************************************************/

#define CM_MOTORFEED_VALUE_MAX (powf(2.0f, (float)CM_MotorFeed_Value_bs) - 1)

//MaxRpm

inline float CM_convert_GetMotorVel_MaxRpm(CM_StmInit_Var_t rpm_scale)
{
	return (CM_MOTORFEED_VALUE_MAX / rpm_scale);
}

inline float CM_convert_GetMotorVel_MaxRpm_N(CM_StmInit_Var_t rpm_scale)
{
	return (CM_MOTORFEED_VALUE_MAX / (rpm_scale/5));
}

//MotorVal2Rpm

inline float CM_convert_GetMotorVel_mtor(CM_StmInit_Var_t rpm_scale, CM_MotorVel_Value_t motor_val, CM_MotorVel_Dir_t motor_dir)
{
	return (motor_dir == 0 ? motor_val/rpm_scale : -motor_val/rpm_scale);
}

inline float CM_convert_GetMotorVel_mtor_N(CM_StmInit_Var_t rpm_scale, CM_MotorVel_Value_t motor_val, CM_MotorVel_Dir_t motor_dir)
{
	return (motor_dir == 0 ? motor_val/(rpm_scale/5) : -motor_val/(rpm_scale/5));
}

inline void CM_convert_GetMotorVel_mtor_M(CM_Vector3f* rpm_out, const CM_StmInit_Var_t rpm_scale[], const CM_GetMotorVel_msg msg_in[])
{
	rpm_out->x = CM_convert_GetMotorVel_mtor(rpm_scale[0], msg_in->motor_A_value, msg_in->motor_A_dir);
	rpm_out->y = CM_convert_GetMotorVel_mtor(rpm_scale[1], msg_in->motor_B_value, msg_in->motor_B_dir);
	rpm_out->z = CM_convert_GetMotorVel_mtor(rpm_scale[2], msg_in->motor_C_value, msg_in->motor_C_dir);
}

inline void CM_convert_GetMotorVel_mtor_NM(CM_Vector3f* rpm_out, const CM_StmInit_Var_t rpm_scale[], const CM_GetMotorVel_msg msg_in[])
{
	rpm_out->x = CM_convert_GetMotorVel_mtor_N(rpm_scale[0], msg_in->motor_A_value, msg_in->motor_A_dir);
	rpm_out->y = CM_convert_GetMotorVel_mtor_N(rpm_scale[1], msg_in->motor_B_value, msg_in->motor_B_dir);
	rpm_out->z = CM_convert_GetMotorVel_mtor_N(rpm_scale[2], msg_in->motor_C_value, msg_in->motor_C_dir);
}

//Rpm2MotorVal

inline void CM_convert_GetMotorVel_rtom(CM_StmInit_Var_t rpm_scale, CM_MotorVel_Value_t* motor_val, CM_MotorVel_Dir_t* motor_dir, float rpm)
{
	*motor_dir = rpm < 0.0f;
	*motor_val = *motor_dir ? -rpm * rpm_scale : rpm * rpm_scale;
}

inline void CM_convert_GetMotorVel_rtom_N(CM_StmInit_Var_t rpm_scale, CM_MotorVel_Value_t* motor_val, CM_MotorVel_Dir_t* motor_dir, float rpm)
{
	*motor_dir = rpm < 0.0f;
	*motor_val = *motor_dir ? -rpm * (rpm_scale/5) : rpm * (rpm_scale/5);
}

inline void CM_convert_GetMotorVel_rtom_M(CM_GetMotorVel_msg* msg_out, const CM_StmInit_Var_t rpm_scale[], const CM_Vector3f* rpm_in)
{
	msg_out->motor_A_dir = (rpm_in->x < 0.0f);
	msg_out->motor_B_dir = (rpm_in->y < 0.0f);
	msg_out->motor_C_dir = (rpm_in->z < 0.0f);

	msg_out->motor_A_value = (CM_MotorVel_Value_t)( (msg_out->motor_A_dir ? -rpm_in->x : rpm_in->x) * rpm_scale[0] );
	msg_out->motor_B_value = (CM_MotorVel_Value_t)( (msg_out->motor_B_dir ? -rpm_in->y : rpm_in->y) * rpm_scale[1] );
	msg_out->motor_C_value = (CM_MotorVel_Value_t)( (msg_out->motor_C_dir ? -rpm_in->z : rpm_in->z) * rpm_scale[2] );
}

inline void CM_convert_GetMotorVel_rtom_NM(CM_GetMotorVel_msg* msg_out, const CM_StmInit_Var_t rpm_scale[], const CM_Vector3f* rpm_in)
{
	float rpm_scale_new[3] = {
		rpm_scale[0] / 5,
		rpm_scale[1] / 5,
		rpm_scale[2] / 5
	};

	CM_convert_GetMotorVel_rtom_M(msg_out, rpm_scale_new, rpm_in);
}

/****************************************************************
	convert CanMessage
*****************************************************************/

#ifdef CM_ENABLE_MSGCONVERT

inline can_msgs::Frame CM_convert_CanMessage_mtof(const CM_CanMessage& msg)
{
	can_msgs::Frame canFrame;
	canFrame.id = (uint32_t)msg.address;
	canFrame.header.stamp = ros::Time::now();
	canFrame.dlc = msg.dataLen;
	memcpy(canFrame.data.begin(), &msg.data, CM_CAN_DLEN_MAX);
	return canFrame;
}

inline CM_CanMessage CM_convert_CanMessage_ftom(const can_msgs::Frame* frame)
{
	CM_CanMessage canMsg;
	canMsg.address = frame->id;
	canMsg.dataLen = frame->dlc;
	memcpy(&canMsg.data, frame->data.begin() ,CM_CAN_DLEN_MAX);
	return canMsg;
}

#endif //CM_ENABLE_MSGCONVERT

#endif //CM_Functions_h_