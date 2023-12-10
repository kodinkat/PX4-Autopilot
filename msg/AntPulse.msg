uint64 timestamp			# Time since system start (microseconds)
uint16 ant_id				# Unique ant id

# Battery
bool has_battery_telemetry
float32 remaining			# From 1 to 0, -1 if unknown
float32 time_remaining_s		# Predicted time in seconds remaining until battery is empty under previous averaged load, NAN if unknown
float32 temperature			# Temperature of the battery. NaN if unknown
uint16 capacity				# Actual capacity of the battery
uint16 average_time_to_empty		# Predicted remaining battery capacity based on the average rate of discharge in min
float32 available_energy            	# The predicted charge or energy remaining in the battery
float32 full_charge_capacity_wh     	# The compensated battery capacity
float32 remaining_capacity_wh       	# The compensated battery capacity remaining
float32 design_capacity             	# The design capacity of the battery
uint16 average_time_to_full         	# The predicted remaining time until the battery reaches full charge, in minutes

# Height sensors
bool has_height_telemetry
float32 rng_vpos			# range sensor height innovation (m) and innovation variance (m**2)
float32 baro_vpos			# barometer height innovation (m) and innovation variance (m**2)


# TOPICS ant_pulse
