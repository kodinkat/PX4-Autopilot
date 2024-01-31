uint64 timestamp			# Time since system start (microseconds)
uint16 ant_id				# Unique ant id

### FailsafeFlags
bool has_fail_safe_flags

# Battery
uint8 fsf_battery_warning                 # Battery warning level
bool fsf_battery_low_remaining_time       # Low battery based on remaining flight time
bool fsf_battery_unhealthy                # Battery unhealthy

# Other
bool fsf_wind_limit_exceeded              # Wind limit exceeded
bool fsf_flight_time_limit_exceeded       # Maximum flight time exceeded

# Failure detector
bool fsf_fd_critical_failure              # Critical failure (attitude/altitude limit exceeded, or external ATS)
bool fsf_fd_esc_arming_failure            # ESC failed to arm
bool fsf_fd_imbalanced_prop               # Imbalanced propeller detected
bool fsf_fd_motor_failure                 # Motor failure


### FailureDetectorStatus
bool has_failure_detector_status

# FailureDetector status
bool fds_fd_roll
bool fds_fd_pitch
bool fds_fd_alt
bool fds_fd_ext
bool fds_fd_arm_escs
bool fds_fd_battery
bool fds_fd_imbalanced_prop
bool fds_fd_motor

float32 fds_imbalanced_prop_metric      # Metric of the imbalanced propeller check (low-passed)
uint16 fds_motor_failure_mask           # Bit-mask with motor indices, indicating critical motor failures


# TOPICS ant_failures
