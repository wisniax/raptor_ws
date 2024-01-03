#ifndef CM_Message_h_
#define CM_Message_h_

#include <stdint.h>

#ifdef __GNUC__
#define CM_GCC_ATTRIBUTE_PACKED __attribute__((packed))
#else
#define CM_GCC_ATTRIBUTE_PACKED
#endif

/****************************************************************
	Typedefs (_t) and bit-field size defs (_bs)
*****************************************************************/

typedef uint32_t CM_Address_t;
typedef uint8_t CM_DataLen_t;
typedef uint8_t CM_DataRaw_t;
#define CM_CAN_DLEN_MAX 8

typedef uint8_t CM_Mode_ContMode_t;
#define CM_Mode_ContMode_bs 2
typedef uint8_t CM_Mode_Unused_t;
#define CM_Mode_Unused_bs 2
typedef uint8_t CM_Mode_Reason_t;
#define CM_Mode_Reason_bs 2

typedef uint8_t CM_NodeErrors_Error_t;
#define CM_NodeErrors_Error_bs 4

typedef uint8_t CM_StmInit_TypeId_t;
#define CM_StmInit_TypeId_bs 8
typedef float CM_StmInit_Var_t;
//#define CM_StmInit_Var_bs 32 // kinda not bit field because float..

typedef uint16_t CM_MotorVel_Value_t;
#define CM_MotorVel_Value_bs 11
typedef uint8_t CM_MotorVel_Dir_t;
#define CM_MotorVel_Dir_bs 1

typedef uint16_t CM_MotorFeed_Value_t;
#define CM_MotorFeed_Value_bs 15
typedef uint8_t CM_MotorFeed_Dir_t;
#define CM_MotorFeed_Dir_bs 1

typedef uint16_t CM_MotorCurr_Value_t;
#define CM_MotorCurr_Value_bs 11
typedef uint8_t CM_MotorCurr_Dir_t;
#define CM_MotorCurr_Dir_bs 1



/****************************************************************
	templates
*****************************************************************/

#define CM_emplace_mode_smgs(prefix,postfix)			\
	CM_Mode_ContMode_t	prefix	##	cont_mode	##	postfix	: CM_Mode_ContMode_bs;	\
	CM_Mode_Unused_t	prefix	##	tbd			##	postfix	: CM_Mode_Unused_bs;	\
	CM_Mode_Reason_t	prefix	##	reason		##	postfix	: CM_Mode_Reason_bs;

#define CM_emplace_motor_vel_smgs(prefix,postfix)	\
	CM_MotorVel_Value_t	prefix	##	value	##	postfix	: CM_MotorVel_Value_bs;	\
	CM_MotorVel_Dir_t	prefix	##	dir		##	postfix	: CM_MotorVel_Dir_bs;

#define CM_emplace_motor_feed_smgs(prefix,postfix)	\
	CM_MotorFeed_Value_t	prefix	##	value	##	postfix	: CM_MotorFeed_Value_bs;	\
	CM_MotorFeed_Dir_t		prefix	##	dir		##	postfix	: CM_MotorVel_Dir_bs;

#define CM_emplace_motor_curr_smgs(prefix,postfix)	\
	CM_MotorCurr_Value_t	prefix	##	value	##	postfix	: CM_MotorCurr_Value_bs;	\
	CM_MotorCurr_Dir_t		prefix	##	dir		##	postfix	: CM_MotorCurr_Dir_bs;



/****************************************************************
	Sub Message Types (_smsg)
*****************************************************************/
#ifdef CM_DEFINE_SMSG

typedef struct CM_GCC_ATTRIBUTE_PACKED
{
	CM_emplace_mode_smgs(, )
} CM_Mode_smsg;

typedef struct CM_GCC_ATTRIBUTE_PACKED
{
	CM_emplace_motor_vel_smgs(, )
} CM_MotorVel_smsg;

typedef struct CM_GCC_ATTRIBUTE_PACKED
{
	CM_emplace_motor_feed_smgs(, )
} CM_MotorFeed_smsg;

typedef struct CM_GCC_ATTRIBUTE_PACKED
{
	CM_emplace_motor_curr_smgs(, )
} CM_MotorCurr_smsg;

#endif



/****************************************************************
	Message Types (_msg)
*****************************************************************/

typedef struct CM_GCC_ATTRIBUTE_PACKED
{
	CM_StmInit_TypeId_t	type_id;
	CM_StmInit_Var_t	var;
} CM_StmInit_msg;

typedef struct CM_GCC_ATTRIBUTE_PACKED
{
	CM_NodeErrors_Error_t unique_err : CM_NodeErrors_Error_bs;
	CM_NodeErrors_Error_t select_err : CM_NodeErrors_Error_bs;
	CM_NodeErrors_Error_t rpm_scale_err : CM_NodeErrors_Error_bs;
	CM_NodeErrors_Error_t motor_a_reg_err : CM_NodeErrors_Error_bs;
	CM_NodeErrors_Error_t motor_b_reg_err : CM_NodeErrors_Error_bs;
	CM_NodeErrors_Error_t motor_c_reg_err : CM_NodeErrors_Error_bs;
} CM_NodeErrors_msg;

typedef struct CM_GCC_ATTRIBUTE_PACKED
{
	CM_emplace_mode_smgs(mode_, )
	CM_emplace_motor_vel_smgs(motor_A_, )
	CM_emplace_motor_vel_smgs(motor_B_, )
	CM_emplace_motor_vel_smgs(motor_C_, )
} CM_SetMotorVel_msg;

typedef struct CM_GCC_ATTRIBUTE_PACKED
{
	CM_emplace_mode_smgs(mode_, )
	CM_emplace_motor_feed_smgs(motor_A_, )
	CM_emplace_motor_feed_smgs(motor_B_, )
	CM_emplace_motor_feed_smgs(motor_C_, )
} CM_GetMotorVel_msg;

typedef struct CM_GCC_ATTRIBUTE_PACKED
{
	CM_emplace_mode_smgs(mode_, )
	CM_emplace_motor_curr_smgs(motor_A_, )
	CM_emplace_motor_curr_smgs(motor_B_, )
	CM_emplace_motor_curr_smgs(motor_C_, )
} CM_GetMotorCurr_msg;



/****************************************************************
	CanMessage
*****************************************************************/

typedef struct
{
	CM_Address_t address;
	CM_DataLen_t dataLen;

	union CM_GCC_ATTRIBUTE_PACKED
	{
		CM_DataRaw_t raw[CM_CAN_DLEN_MAX]; //raw data representation

		CM_StmInit_msg stm_init; 

		CM_NodeErrors_msg node_errors;

		CM_SetMotorVel_msg set_motor_vel;

		CM_GetMotorVel_msg get_motor_vel;

		CM_GetMotorCurr_msg get_motor_curr;

		#ifdef CM_DEFINE_SMSG

		CM_Mode_smsg mode;

		#endif

	} data;
} CM_CanMessage;
#endif //CM_Message_h_