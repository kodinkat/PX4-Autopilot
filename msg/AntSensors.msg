uint64 timestamp			# Time since system start (microseconds)
uint16 ant_system_id			# Unique ant system id


### SensorAccel
bool has_sensor_accel
uint32 sa_device_id          # unique device ID for the sensor that does not change between power cycles

float32 sa_x                 # acceleration in the FRD board frame X-axis in m/s^2
float32 sa_y                 # acceleration in the FRD board frame Y-axis in m/s^2
float32 sa_z                 # acceleration in the FRD board frame Z-axis in m/s^2

float32 sa_temperature       # temperature in degrees Celsius


### SensorBaro
bool has_sensor_baro
uint32 sb_device_id          # unique device ID for the sensor that does not change between power cycles

float32 sb_pressure          # static pressure measurement in Pascals

float32 sb_temperature       # temperature in degrees Celsius


### SensorGyro
bool has_sensor_gyro
uint32 sg_device_id          # unique device ID for the sensor that does not change between power cycles

float32 sg_x                 # angular velocity in the FRD board frame X-axis in rad/s
float32 sg_y                 # angular velocity in the FRD board frame Y-axis in rad/s
float32 sg_z                 # angular velocity in the FRD board frame Z-axis in rad/s

float32 sg_temperature       # temperature in degrees Celsius


### SensorHygrometer
bool has_sensor_hygrometer
uint32 sh_device_id          # unique device ID for the sensor that does not change between power cycles

float32  sh_temperature      # Temperature provided by sensor (Celsius)

float32 sh_humidity          # Humidity provided by sensor


### SensorMag
bool has_sensor_mag
uint32 sm_device_id          # unique device ID for the sensor that does not change between power cycles

float32 sm_x                 # magnetic field in the FRD board frame X-axis in Gauss
float32 sm_y                 # magnetic field in the FRD board frame Y-axis in Gauss
float32 sm_z                 # magnetic field in the FRD board frame Z-axis in Gauss

float32 sm_temperature       # temperature in degrees Celsius


### SensorOpticalFlow
bool has_sensor_optical_flow
uint32 sof_device_id               # unique device ID for the sensor that does not change between power cycles

float32[2] sof_pixel_flow          # (radians) optical flow in radians where a positive value is produced by a RH rotation about the body axis

float32[3] sof_delta_angle         # (radians) accumulated gyro radians where a positive value is produced by a RH rotation about the body axis. Set to NaN if flow sensor does not have 3-axis gyro data.
bool sof_delta_angle_available

float32 sof_distance_m             # (meters) Distance to the center of the flow field
bool sof_distance_available

uint32 sof_integration_timespan_us # (microseconds) accumulation timespan in microseconds

uint8 sof_quality                  # quality, 0: bad quality, 255: maximum quality

uint32 sof_error_count

float32 sof_max_flow_rate          # (radians/s) Magnitude of maximum angular which the optical flow sensor can measure reliably

float32 sof_min_ground_distance    # (meters) Minimum distance from ground at which the optical flow sensor operates reliably
float32 sof_max_ground_distance    # (meters) Maximum distance from ground at which the optical flow sensor operates reliably


# TOPICS ant_sensors
