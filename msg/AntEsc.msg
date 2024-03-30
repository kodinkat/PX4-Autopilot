uint64 timestamp			# Time since system start (microseconds)
uint16 ant_system_id			# Unique ant system id

### EscReport
bool has_esc_report
uint32 er_esc_errorcount					# Number of reported errors by ESC - if supported
int32 er_esc_rpm						# Motor RPM, negative for reverse rotation [RPM] - if supported
float32 er_esc_voltage					# Voltage measured from current ESC [V] - if supported
float32 er_esc_current					# Current measured from current ESC [A] - if supported
float32 er_esc_temperature					# Temperature measured from current ESC [degC] - if supported
uint8 er_esc_address					# Address of current ESC (in most cases 1-8 / must be set by driver)
uint8 er_esc_cmdcount					# Counter of number of commands

uint8 er_esc_state					# State of ESC - depend on Vendor

uint8 er_actuator_function				# actuator output function (one of Motor1...MotorN)

uint16 er_failures					# Bitmask to indicate the internal ESC faults
int8 er_esc_power					# Applied power 0-100 in % (negative values reserved)


### EscStatus
bool has_esc_status
uint8 es_esc_count						# number of connected ESCs
uint8 es_esc_connectiontype				# how ESCs connected to the system

uint8 es_esc_online_flags

uint8 es_esc_armed_flags					# Bitmask indicating which ESC is armed. For ESC's where the arming state is not known (returned by the ESC), the arming bits should always be set.

# EscReport[8] es_esc


# TOPICS ant_esc
