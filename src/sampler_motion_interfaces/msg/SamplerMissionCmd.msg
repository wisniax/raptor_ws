uint8 mission_cmd
float32 platform_movement
float32 drill_movement
float32 drill_action
float32 container_degrees_a
float32 container_degrees_b
float32 vacuum_suction # -1:1 range one way to suck other to...
float32 vacuum_a # on-off some motor A (this one actually exists)
float32 vacuum_b # on-off some motor B (this one does not exist but anyway)

# Mission commands
uint8 START = 0
uint8 STOP  = 1
uint8 ABORT = 2
uint8 RESTART = 3
uint8 RESTART_DEEP = 4
uint8 RESTART_SURFACE = 5
