std_msgs/Header header
float32 weight_a
float32 weight_b
float32 weight_c
float32 ph
float32 platform_pos
float32 drill_pos
float32 container_a_pos
float32 drill_rot_vel


uint8 mission_state
uint8 goal_state
uint8 joint_id

# States
uint8 STATE_IDLE = 0
uint8 STATE_CALIBRATE_PLATFORM = 1
uint8 STATE_CALIBRATE_DRILL = 2
uint8 STATE_MOVE_PLATFORM_DOWN = 3
uint8 STATE_DRILLING = 4
uint8 STATE_MOVE_DRILL_UP = 5
uint8 STATE_MOVE_PLATFORM_UP = 6
uint8 STATE_MEASUREMENTS = 7
uint8 STATE_ABORT = 8
uint8 STATE_DONE = 9

# Goal States
uint8 GOAL_IDLE=0
uint8 GOAL_SENT=1
uint8 GOAL_ACCEPTED=2
uint8 GOAL_EXECUTING=3
uint8 GOAL_SUCCEEDED=4
uint8 GOAL_CANCELED=5
uint8 GOAL_FAILED=6

# Joint ids
uint8 PLATFORM = 1
uint8 DRILL = 2
uint8 CONTAINER_A = 3
uint8 ROTOR = 4


