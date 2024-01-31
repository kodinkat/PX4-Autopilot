uint64 timestamp			# Time since system start (microseconds)
uint16 ant_id				# Unique ant id

### ActuatorArmed
bool has_actuator_armed
bool aa_armed		# Set to true if system is armed
bool aa_prearmed		# Set to true if the actuator safety is disabled but motors are not armed
bool aa_ready_to_arm	# Set to true if system is ready to be armed
bool aa_lockdown		# Set to true if actuators are forced to being disabled (due to emergency or HIL)
bool aa_manual_lockdown    # Set to true if manual throttle kill switch is engaged
bool aa_force_failsafe	# Set to true if the actuators are forced to the failsafe position
bool aa_in_esc_calibration_mode # IO/FMU should ignore messages from the actuator controls topics

### ActuatorMotors
bool has_actuator_motors
uint16 am_reversible_flags     # bitset which motors are configured to be reversible
float32[12] am_control # range: [-1, 1], where 1 means maximum positive thrust,
                    # -1 maximum negative (if not supported by the output, <0 maps to NaN),
                    # and NaN maps to disarmed (stop the motors)



# TOPICS ant_actuators
