uint64 timestamp			# Time since system start (microseconds)
uint16 ant_system_id			# Unique ant system id


### LandingGear
bool has_landing_gear
int8 lg_landing_gear


### VehicleLandDetected
bool has_vehicle_land_detected
bool vld_freefall		# true if vehicle is currently in free-fall
bool vld_ground_contact	# true if vehicle has ground contact but is not landed (1. stage)
bool vld_maybe_landed	# true if the vehicle might have landed (2. stage)
bool vld_landed		# true if vehicle is currently landed on the ground (3. stage)

bool vld_in_ground_effect # indicates if from the perspective of the landing detector the vehicle might be in ground effect (baro). This flag will become true if the vehicle is not moving horizontally and is descending (crude assumption that user is landing).
bool vld_in_descend

bool vld_has_low_throttle

bool vld_vertical_movement
bool vld_horizontal_movement
bool vld_rotational_movement

bool vld_close_to_ground_or_skipped_check

bool vld_at_rest



# TOPICS ant_landing
