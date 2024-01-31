uint64 timestamp			# Time since system start (microseconds)
uint16 ant_id				# Unique ant id

### PowerButtonState
bool has_power_button_state
uint8 pbs_event                                 # one of PWR_BUTTON_STATE_*


### PowerMonitor
bool has_power_monitor
float32 pm_voltage_v			# Voltage in volts, 0 if unknown
float32 pm_current_a		    # Current in amperes, -1 if unknown
float32 pm_power_w		        # power in watts, -1 if unknown

### SystemPower
bool has_system_power
float32 sp_voltage5v_v		# peripheral 5V rail voltage
uint8 sp_usb_connected		# USB is connected when 1
uint8 sp_usb_valid 		# USB is valid when 1


# TOPICS ant_power
