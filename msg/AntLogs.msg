uint64 timestamp			# Time since system start (microseconds)
uint16 ant_system_id			# Unique ant system id


### LogMessage
bool has_log_message
uint8 lm_severity # log level (same as in the linux kernel, starting with 0)
char[127] lm_text


### MavlinkLog
bool has_mavlink_log
char[127] ml_text
uint8 ml_severity # log level (same as in the linux kernel, starting with 0)


# TOPICS ant_logs
