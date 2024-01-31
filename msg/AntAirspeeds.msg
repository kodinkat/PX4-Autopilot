uint64 timestamp			# Time since system start (microseconds)
uint16 ant_id				# Unique ant id

### Airspeed
bool has_airspeed
float32 as_indicated_airspeed_m_s   # indicated airspeed in m/s
float32 as_true_airspeed_m_s        # true filtered airspeed in m/s
float32 as_air_temperature_celsius  # air temperature in degrees Celsius, -1000 if unknown
float32 as_confidence               # confidence value from 0 to 1 for this sensor

### AirspeedValidated
bool has_airspeed_validated
float32 asv_indicated_airspeed_m_s			# indicated airspeed in m/s (IAS), set to NAN if invalid
float32 asv_calibrated_airspeed_m_s     		# calibrated airspeed in m/s (CAS, accounts for instrumentation errors), set to NAN if invalid
float32 asv_true_airspeed_m_s			# true filtered airspeed in m/s (TAS), set to NAN if invalid

float32 asv_calibrated_ground_minus_wind_m_s 	# CAS calculated from groundspeed - windspeed, where windspeed is estimated based on a zero-sideslip assumption, set to NAN if invalid
float32 asv_true_ground_minus_wind_m_s 		# TAS calculated from groundspeed - windspeed, where windspeed is estimated based on a zero-sideslip assumption, set to NAN if invalid

bool asv_airspeed_sensor_measurement_valid 		# True if data from at least one airspeed sensor is declared valid.

int8 asv_selected_airspeed_index 			# 1-3: airspeed sensor index, 0: groundspeed-windspeed, -1: airspeed invalid

### AirspeedWind
bool has_airspeed_wind
float32 asw_windspeed_north		# Wind component in north / X direction (m/sec)
float32 asw_windspeed_east		# Wind component in east / Y direction (m/sec)

float32 asw_variance_north		# Wind estimate error variance in north / X direction (m/sec)**2 - set to zero (no uncertainty) if not estimated
float32 asw_variance_east		# Wind estimate error variance in east / Y direction (m/sec)**2 - set to zero (no uncertainty) if not estimated

float32 asw_tas_innov 		# True airspeed innovation
float32 asw_tas_innov_var 		# True airspeed innovation variance

float32 asw_tas_scale_raw 		# Estimated true airspeed scale factor (not validated)
float32 asw_tas_scale_raw_var 	# True airspeed scale factor variance

float32 asw_tas_scale_validated	# Estimated true airspeed scale factor after validation

float32 asw_beta_innov 		# Sideslip measurement innovation
float32 asw_beta_innov_var 		# Sideslip measurement innovation variance

uint8 asw_source			# source of wind estimate

### Wind
bool has_wind
float32 wind_windspeed_north		# Wind component in north / X direction (m/sec)
float32 wind_windspeed_east		# Wind component in east / Y direction (m/sec)

float32 wind_variance_north		# Wind estimate error variance in north / X direction (m/sec)**2 - set to zero (no uncertainty) if not estimated
float32 wind_variance_east		# Wind estimate error variance in east / Y direction (m/sec)**2 - set to zero (no uncertainty) if not estimated

float32 wind_tas_innov 		# True airspeed innovation
float32 wind_tas_innov_var 		# True airspeed innovation variance

float32 wind_beta_innov 		# Sideslip measurement innovation
float32 wind_beta_innov_var 		# Sideslip measurement innovation variance


# TOPICS ant_airspeeds
