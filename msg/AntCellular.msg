uint64 timestamp			# Time since system start (microseconds)
uint16 ant_system_id			# Unique ant system id


### CellularStatus
bool has_cellular_status
uint16 cs_status 	# Status bitmap 1: Roaming is active
uint8 cs_failure_reason #Failure reason when status in in CELLUAR_STATUS_FAILED
uint8 cs_type 	# Cellular network radio type 0: none 1: gsm 2: cdma 3: wcdma 4: lte
uint8 cs_quality	# Cellular network RSSI/RSRP in dBm, absolute value
uint16 cs_mcc	# Mobile country code. If unknown, set to: UINT16_MAX
uint16 cs_mnc	# Mobile network code. If unknown, set to: UINT16_MAX
uint16 cs_lac	# Location area code. If unknown, set to: 0



# TOPICS ant_cellular
