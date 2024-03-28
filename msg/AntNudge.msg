uint64 timestamp			# Time since system start (microseconds)
uint16 ant_system_id			# Unique ant system id

uint16 nudge_system_id			# Unique nudge system id

uint8 nudge_origin
uint8 NUDGE_ORIGIN_MANUAL = 0
uint8 NUDGE_ORIGIN_AUTO = 1

uint8 nudge_nav_type
uint8 NUDGE_NAV_TYPE_ARM = 0
uint8 NUDGE_NAV_TYPE_DISARM = 1
uint8 NUDGE_NAV_TYPE_TAKEOFF = 2
uint8 NUDGE_NAV_TYPE_LAND = 3
uint8 NUDGE_NAV_TYPE_TURN_RIGHT = 4
uint8 NUDGE_NAV_TYPE_TURN_LEFT = 5
uint8 NUDGE_NAV_TYPE_FORWARD = 6
uint8 NUDGE_NAV_TYPE_BACKWARD = 7
uint8 NUDGE_NAV_TYPE_UP = 8
uint8 NUDGE_NAV_TYPE_DOWN = 9
uint8 NUDGE_NAV_TYPE_RIGHT = 10
uint8 NUDGE_NAV_TYPE_LEFT = 11

float32 nnt_takeoff_altitude

float32 nnt_turn_right_radians
float32 nnt_turn_left_radians

float32 nnt_forward_distance
float32 nnt_backward_distance

float32 nnt_up_distance
float32 nnt_down_distance

float32 nnt_right_distance
float32 nnt_left_distance

# TOPICS ant_nudge
