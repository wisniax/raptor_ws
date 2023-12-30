#ifndef CM_Consts_h_
#define CM_Consts_h_

//macro for generating enum values from 0x0 to 0xE
#define CM_gen_enum_variant_0_E(prefix, startvalue)\
	prefix ## 0 = startvalue + 0x0, \
	prefix ## 1 = startvalue + 0x1, \
	prefix ## 2 = startvalue + 0x2, \
	prefix ## 3 = startvalue + 0x3, \
	prefix ## 4 = startvalue + 0x4, \
	prefix ## 5 = startvalue + 0x5, \
	prefix ## 6 = startvalue + 0x6, \
	prefix ## 7 = startvalue + 0x7, \
	prefix ## 8 = startvalue + 0x8, \
	prefix ## 9 = startvalue + 0x9, \
	prefix ## A = startvalue + 0xA, \
	prefix ## B = startvalue + 0xB, \
	prefix ## C = startvalue + 0xC, \
	prefix ## D = startvalue + 0xD, \
	prefix ## E = startvalue + 0xE,

//macro for generating enum values from 0x0 to 0xF
#define CM_gen_enum_variant_0_F(prefix, startvalue)\
	prefix ## 0 = startvalue + 0x0, \
	prefix ## 1 = startvalue + 0x1, \
	prefix ## 2 = startvalue + 0x2, \
	prefix ## 3 = startvalue + 0x3, \
	prefix ## 4 = startvalue + 0x4, \
	prefix ## 5 = startvalue + 0x5, \
	prefix ## 6 = startvalue + 0x6, \
	prefix ## 7 = startvalue + 0x7, \
	prefix ## 8 = startvalue + 0x8, \
	prefix ## 9 = startvalue + 0x9, \
	prefix ## A = startvalue + 0xA, \
	prefix ## B = startvalue + 0xB, \
	prefix ## C = startvalue + 0xC, \
	prefix ## D = startvalue + 0xD, \
	prefix ## E = startvalue + 0xE, \
	prefix ## F = startvalue + 0xF,



/****************************************************************
	
	Address enums

*****************************************************************/

#define CM_ADDRESS_FAMILY_MASK	0x7F0	// Mask for CM_AddressFamily in CM_Address_t
#define CM_ADDRESS_MASK			0x00F	// Mask for CM_Address in CM_Address_t

#define CM_ADDRESS_FAMILY_COUNT 7
typedef enum
{
	CM_ADDRESS_FAMILY_UNNAMED = 0x00,					// Well. I really don't know.
	CM_ADDRESS_FAMILY_ERROR = 0x10,						// RX ERRORS (ROS <-- CAN)
	CM_ADDRESS_FAMILY_INIT = 0x20,						// TX Init (ROS --> CAN)
	CM_ADDRESS_FAMILY_STATUS = 0x30,					// RX (ROS <-- CAN)
	CM_ADDRESS_FAMILY_MOTOR_CONTROL = 0x40,				// TX (ROS --> CAN)
	CM_ADDRESS_FAMILY_ENCODER_VELOCITY_FEEDBACK = 0x50,	// RX (ROS <-- CAN)
	CM_ADDRESS_FAMILY_ENCODER_DISTANCE_TRAVELED = 0x60,	// RX (ROS <-- CAN)
	CM_ADDRESS_FAMILY_MAX = 0x70	// any valid value should be lower
} CM_AddressFamily;

#define CM_ADDRESS_COUNT 4
typedef enum
{
	CM_ADDRESS_STM_LEFT = 0x5,			// direction defined by Family (ROS <-> CAN)
	CM_ADDRESS_STM_RIGHT = 0x6,			// direction defined by Family (ROS <-> CAN)
	CM_ADDRESS_STM_ARM_AXIS_123 = 0x7,	// direction defined by Family (ROS <-> CAN)
	CM_ADDRESS_STM_ARM_AXIS_456 = 0x8,	// direction defined by Family (ROS <-> CAN)
	CM_ADDRESS_MAX = 0x9 // any valid value should be lower
} CM_Address;



/****************************************************************
	
	mode Reason

*****************************************************************/

typedef enum
{
	CM_MODE_REASON_UPDATE,
	CM_MODE_REASON_UPKEEP
} CM_Mode_Reason;

/****************************************************************
		mode cont_mode for set_motor_vel message 
*****************************************************************/

typedef enum
{
	CM_SETMOTORVEL_CONTMODE_RPM,		// treat motor_vel_smsg::value as RPM target
	CM_SETMOTORVEL_CONTMODE_UNUSED0,
	CM_SETMOTORVEL_CONTMODE_UNUSED1,
	CM_SETMOTORVEL_CONTMODE_PWM,		// treat motor_vel_smsg::value as PWM fill rate
} CM_SetMotorVel_ContMode;

/****************************************************************
		mode cont_mode for get_motor_vel message
*****************************************************************/

typedef enum
{
	CM_GETMOTORVEL_CONTMODE_RPM,			// treat motor_feed_smsg::value as RPM value
	CM_GETMOTORVEL_CONTMODE_RPM_NEWSCALE,	// treat motor_feed_smsg::value as RPM value (in new scale)
	CM_GETMOTORVEL_CONTMODE_UNUSED0,
	CM_GETMOTORVEL_CONTMODE_UNUSED1,
} CM_GetMotorVel_ContMode;



/****************************************************************
	
	stm_Init::type_id Families

*****************************************************************/

#define CM_STMINIT_TYPEID_FAMILY_MASK	0xF0	//Mask for CM_StmInit_TypeIdFamily in CM_StmInit_TypeId_t
#define CM_STMINIT_TYPEID_MASK			0x0F	//Mask for CM_StmInit_TypeId family member in CM_StmInit_TypeId_t

#define CM_STMINIT_TYPEID_FAMILY_COUNT 5
typedef enum
{
	CM_STMINIT_TYPEID_FAMILY_SETUP = 0x00,			// Family for basic setup. (see CM_StmInit_TypeId_Setup)
	CM_STMINIT_TYPEID_FAMILY_MOTORCONTROL = 0x10,	// Family for rpm_scale setup (see CM_StmInit_TypeId_RpmScale)
	CM_STMINIT_TYPEID_FAMILY_MOTOR_A = 0x20,		// Family for motor A driver params (see CM_GenericMotorParam)
	CM_STMINIT_TYPEID_FAMILY_MOTOR_B = 0x30,		// Family for motor B driver params	(see CM_GenericMotorParam)
	CM_STMINIT_TYPEID_FAMILY_MOTOR_C = 0x40,		// Family for motor C driver params (see CM_GenericMotorParam)
	CM_STMINIT_TYPEID_FAMILY_MAX = 0x50 // any valid value should be lower
} CM_StmInit_TypeIdFamily;

/****************************************************************
		stm_Init::type_id Setup Family
*****************************************************************/

#define CM_STMINIT_TYPEID_SETUP_COUNT 2
typedef enum
{
	CM_STMINIT_TYPEID_SETUP_DEINIT = 0x0,		// Deinitialization
	CM_STMINIT_TYPEID_SETUP_CONTROLMODE = 0x1,	// Control mode change (see CM_StmInit_ControlMode)
	CM_STMINIT_TYPEID_SETUP_MAX = 0x2// any valid value should be lower
} CM_StmInit_TypeId_Setup;

#define CM_STMINIT_CONTROLMODE_COUNT 4
typedef enum
{
	CM_STMINIT_CONTROLMODE_STOP,		// stop. Whatever u doing. JUST STOP
	CM_STMINIT_CONTROLMODE_ROVER,		// driving around mode
	CM_STMINIT_CONTROLMODE_ARM,			// poking things mode
	CM_STMINIT_CONTROLMODE_AUTONOMY,	// do-things mode
	CM_STMINIT_CONTROLMODE_MAX // any valid value should be lower
} CM_StmInit_ControlMode;

/****************************************************************
		stm_Init::type_id MotorControl Family
*****************************************************************/

#define CM_STMINIT_TYPEID_MOTORCONTROL_COUNT 2
typedef enum
{
	CM_STMINIT_TYPEID_MOTORCONTROL_COMMAND = 0x0,	// Sets rpm_scale for motor_vel
	CM_STMINIT_TYPEID_MOTORCONTROL_FEEDBACK = 0x1,	// Sets rpm_scale for motor_feed
	CM_STMINIT_TYPEID_MOTORCONTROL_MAX = 0x2 // any valid value should be lower
} CM_StmInit_TypeId_MotorControl;

/****************************************************************
		stm_Init::type_id Motor (A,B,C) Family
*****************************************************************/

typedef enum
{
	CM_gen_enum_variant_0_F(CM_GENERICMOTORPARAM_, 0x0)
	CM_GENERICMOTORPARAM_MAX = 0x10// any valid value should be lower
} CM_GenericMotorParam;



/****************************************************************

	node_errors

*****************************************************************/

typedef enum
{
	CM_NODEERRORS_UNIQUEERR_OK,
	CM_NODEERRORS_UNIQUEERR_INVALID_STATUS,
	CM_NODEERRORS_UNIQUEERR_MAX // any valid value should be lower
} CM_NodeErrors_UniqueErr;

#define CM_NODEERRORS_SELECTERR_OK 0x0
typedef enum
{
	CM_NODEERRORS_SELECTERR_FLAG_RPMSCALE = 0x1,		// Error in CM_STMINIT_TYPEID_FAMILY_MOTORCONTROL
	CM_NODEERRORS_SELECTERR_FLAG_MOTORDRIVER_A = 0x2,	// Error in CM_STMINIT_TYPEID_FAMILY_MOTOR_A
	CM_NODEERRORS_SELECTERR_FLAG_MOTORDRIVER_B = 0x4,	// Error in CM_STMINIT_TYPEID_FAMILY_MOTOR_B
	CM_NODEERRORS_SELECTERR_FLAG_MOTORDRIVER_C = 0x8	// Error in CM_STMINIT_TYPEID_FAMILY_MOTOR_C
} CM_NodeErrors_SelectErr_Flags;

#define CM_NODEERRORS_RPMSCALE_COUNT 2
typedef enum
{
	CM_NODEERRORS_RPMSCALE_MISSING_IN_COMMAND = 0x0,	// rpm_scale not set for message set_motor_vel
	CM_NODEERRORS_RPMSCALE_MISSING_IN_FEEDBACK = 0x1,	// rpm_scale not set for message get_motor_vel
	CM_NODEERRORS_RPMSCALE_MISSING_IN_ALL = 0xF			// rpm_scale not set for message set_motor_vel and get_motor_vel
} CM_NodeErrors_RpmScale;

typedef enum
{
	CM_gen_enum_variant_0_E(CM_NODEERRORS_MOTORDRIVER_PARAM_MISSING_,0x0)
	CM_NODEERRORS_MOTORDRIVER_PARAM_MISSING_ALL = 0xF
} CM_NodeErrors_MotorDriver;

#endif //CM_Consts_h_