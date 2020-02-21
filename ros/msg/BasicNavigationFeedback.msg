uint8 SUCCESS = 0
uint8 REACHED_WP = 1
uint8 CANCELLED = 2
uint8 FAILURE_OBSTACLES = 3
uint8 FAILURE_NO_CURRENT_POSE = 4
uint8 FAILURE_EMPTY_PLAN = 5
uint8 FAILURE_INVALID_PLAN = 6

uint8 status
geometry_msgs/Pose reached_wp
uint16 remaining_path_length
