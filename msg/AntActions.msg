uint64 timestamp			# Time since system start (microseconds)
uint16 ant_system_id			# Unique ant system id

### ActionRequest
bool has_action_request
uint8 ar_action # what action is requested
uint8 ar_source # how the request was triggered
uint8 ar_mode # for ACTION_SWITCH_MODE what mode is requested according to vehicle_status_s::NAVIGATION_STATE_*

### TakeoffStatus
bool has_takeoff_status
uint8 tos_takeoff_state
float32 tos_tilt_limit # limited tilt feasibility during takeoff, contains maximum tilt otherwise


# TOPICS ant_actions
