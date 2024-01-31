uint64 timestamp			# Time since system start (microseconds)
uint16 ant_id				# Unique ant id


### RadioStatus
bool has_radio_status
uint8 rs_rssi				# local signal strength
uint8 rs_remote_rssi			# remote signal strength

uint8 rs_noise				# background noise level
uint8 rs_remote_noise			# remote background noise level


# TOPICS ant_radio
