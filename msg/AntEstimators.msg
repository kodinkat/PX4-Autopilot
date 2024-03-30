uint64 timestamp			# Time since system start (microseconds)
uint16 ant_system_id			# Unique ant system id


### EstimatorInnovations
bool has_estimator_innovations

# Height sensors
float32 ei_rng_vpos	# range sensor height innovation (m) and innovation variance (m**2)
float32 ei_baro_vpos	# barometer height innovation (m) and innovation variance (m**2)

# Optical flow
float32[2] ei_flow		# flow innvoation (rad/sec) and innovation variance ((rad/sec)**2)
float32[2] ei_terr_flow	# flow innvoation (rad/sec) and innovation variance computed by the terrain estimator ((rad/sec)**2)

# Various
float32 ei_heading		# heading innovation (rad) and innovation variance (rad**2)
float32[3] ei_mag_field	# earth magnetic field innovation (Gauss) and innovation variance (Gauss**2)
float32[3] ei_gravity	# gravity innovation from accelerometerr vector (m/s**2)
float32[2] ei_drag		# drag specific force innovation (m/sec**2) and innovation variance ((m/sec)**2)
float32 ei_airspeed	# airspeed innovation (m/sec) and innovation variance ((m/sec)**2)
float32 ei_beta		# synthetic sideslip innovation (rad) and innovation variance (rad**2)
float32 ei_hagl		# height of ground innovation (m) and innovation variance (m**2)
float32 ei_hagl_rate	# height of ground rate innovation (m/s) and innovation variance ((m/s)**2)


### EstimatorStatusFlags
bool has_estimator_status_flags

# filter control status
uint32 esf_control_status_changes # number of filter control status (cs) changes
bool esf_cs_in_air                #  7 - true when the vehicle is airborne

# fault status
uint32 esf_fault_status_changes   # number of filter fault status (fs) changes

# innovation test failures
uint32 esf_innovation_fault_status_changes    # number of innovation fault status (reject) changes


# TOPICS ant_estimators
