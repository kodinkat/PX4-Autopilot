uint64 timestamp			# Time since system start (microseconds)
uint16 ant_system_id			# Unique ant system id

### BatteryStatus
bool has_battery_status
bool bs_connected				# Whether or not a battery is connected, based on a voltage threshold
float32 bs_voltage_v			# Battery voltage in volts, 0 if unknown
float32 bs_voltage_filtered_v	# Battery voltage in volts, filtered, 0 if unknown
float32 bs_current_a			# Battery current in amperes, -1 if unknown
float32 bs_current_filtered_a	# Battery current in amperes, filtered, 0 if unknown
float32 bs_current_average_a	# Battery current average in amperes, -1 if unknown
float32 bs_discharged_mah		# Discharged amount in mAh, -1 if unknown
float32 bs_remaining			# From 1 to 0, -1 if unknown
float32 bs_scale				# Power scaling factor, >= 1, or -1 if unknown
float32 bs_time_remaining_s	# predicted time in seconds remaining until battery is empty under previous averaged load, NAN if unknown
float32 bs_temperature			# temperature of the battery. NaN if unknown
uint8 bs_cell_count			# Number of cells, 0 if unknown

uint8 bs_source				# Battery source
uint8 bs_priority				# Zero based priority is the connection on the Power Controller V1..Vn AKA BrickN-1
uint16 bs_capacity				# actual capacity of the battery
uint16 bs_cycle_count			# number of discharge cycles the battery has experienced
uint16 bs_average_time_to_empty	# predicted remaining battery capacity based on the average rate of discharge in min
uint16 bs_serial_number		# serial number of the battery pack
uint16 bs_manufacture_date		# manufacture date, part of serial number of the battery pack. Formatted as: Day + Month×32 + (Year–1980)×512
uint16 bs_state_of_health		# state of health. FullChargeCapacity/DesignCapacity, 0-100%.
uint16 bs_max_error			# max error, expected margin of error in % in the state-of-charge calculation with a range of 1 to 100%
uint8 bs_id					# ID number of a battery. Should be unique and consistent for the lifetime of a vehicle. 1-indexed.
uint16 bs_interface_error		# interface error counter

float32[14] bs_voltage_cell_v		# Battery individual cell voltages, 0 if unknown
float32 bs_max_cell_voltage_delta	# Max difference between individual cell voltages

bool bs_is_powering_off			# Power off event imminent indication, false if unknown

uint16 bs_faults		# Smart battery supply status/fault flags (bitmask) for health indication.
uint32 bs_custom_faults 	# Bitmask indicating smart battery internal manufacturer faults, those are not user actionable.
uint8 bs_warning		# Current battery warning
uint8 bs_mode 		# Battery mode. Note, the normal operation mode

float32 bs_average_power               # The average power of the current discharge
float32 bs_available_energy            # The predicted charge or energy remaining in the battery
float32 bs_full_charge_capacity_wh     # The compensated battery capacity
float32 bs_remaining_capacity_wh       # The compensated battery capacity remaining
float32 bs_design_capacity             # The design capacity of the battery
uint16 bs_average_time_to_full         # The predicted remaining time until the battery reaches full charge, in minutes
uint16 bs_over_discharge_count         # Number of battery overdischarge
float32 bs_nominal_voltage             # Nominal voltage of the battery pack


# TOPICS ant_battery
