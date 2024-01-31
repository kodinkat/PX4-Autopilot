uint64 timestamp			# Time since system start (microseconds)
uint16 ant_id				# Unique ant id

### HoverThrustEstimate
bool has_hover_thrust_estimate
float32 hte_hover_thrust		# estimated hover thrust [0.1, 0.9]
float32 hte_hover_thrust_var	# estimated hover thrust variance

float32 hte_accel_innov		# innovation of the last acceleration fusion
float32 hte_accel_innov_var		# innovation variance of the last acceleration fusion
float32 hte_accel_innov_test_ratio	# normalized innovation squared test ratio

float32 hte_accel_noise_var		# vertical acceleration noise variance estimated form innovation residual

bool hte_valid


# TOPICS ant_hovering
