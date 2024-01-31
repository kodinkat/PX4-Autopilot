uint64 timestamp			# Time since system start (microseconds)
uint16 ant_id				# Unique ant id


### CollisionReport
bool has_collision_report
uint8 cr_src
uint32 cr_id
uint8 cr_action
uint8 cr_threat_level
float32 cr_time_to_minimum_delta
float32 cr_altitude_minimum_delta
float32 cr_horizontal_minimum_delta

# TOPICS ant_collisions
