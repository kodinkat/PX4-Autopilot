uint64 timestamp			# Time since system start (microseconds)
uint16 ant_system_id			# Unique ant system id


### HealthReport
bool has_health_report
uint64 hr_health_is_present_flags         # flags for each health_component_t
uint64 hr_health_warning_flags
uint64 hr_health_error_flags
# A component is required but missing, if present==0 and error==1

uint64 hr_arming_check_warning_flags
uint64 hr_arming_check_error_flags

# TOPICS ant_health
