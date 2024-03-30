uint64 timestamp			# Time since system start (microseconds)
uint16 ant_system_id			# Unique ant system id


### DistanceSensor
bool has_distance_sensor
uint32 ds_device_id		# unique device ID for the sensor that does not change between power cycles

float32 ds_min_distance		# Minimum distance the sensor can measure (in m)
float32 ds_max_distance		# Maximum distance the sensor can measure (in m)
float32 ds_current_distance	# Current distance reading (in m)
float32 ds_variance		# Measurement variance (in m^2), 0 for unknown / invalid readings
int8 ds_signal_quality		# Signal quality in percent (0...100%), where 0 = invalid signal, 100 = perfect signal, and -1 = unknown signal quality.

uint8 ds_type			# Type from MAV_DISTANCE_SENSOR enum

float32 ds_h_fov # Sensor horizontal field of view (rad)
float32 ds_v_fov # Sensor vertical field of view (rad)
float32[4] ds_q # Quaterion sensor orientation with respect to the vehicle body frame to specify the orientation ROTATION_CUSTOM

uint8 ds_orientation		# Direction the sensor faces from MAV_SENSOR_ORIENTATION enum


### ObstacleDistance
bool has_obstacle_distance
uint8 od_frame		#Coordinate frame of reference for the yaw rotation and offset of the sensor data. Defaults to MAV_FRAME_GLOBAL, which is North aligned. For body-mounted sensors use MAV_FRAME_BODY_FRD, which is vehicle front aligned.
uint8 od_sensor_type # Type from MAV_DISTANCE_SENSOR enum.

uint16[72] od_distances # Distance of obstacles around the UAV with index 0 corresponding to local North. A value of 0 means that the obstacle is right in front of the sensor. A value of max_distance +1 means no obstacle is present. A value of UINT16_MAX for unknown/not used. In a array element, one unit corresponds to 1cm.

float32 od_increment # Angular width in degrees of each array element.

uint16 od_min_distance # Minimum distance the sensor can measure in centimeters.
uint16 od_max_distance # Maximum distance the sensor can measure in centimeters.

float32 od_angle_offset # Relative angle offset of the 0-index element in the distances array. Value of 0 corresponds to forward. Positive values are offsets to the right.



# TOPICS ant_distances
