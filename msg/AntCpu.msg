uint64 timestamp			# Time since system start (microseconds)
uint16 ant_id				# Unique ant id


### Cpuload
bool has_cpu_load
float32 cl_load                    # processor load from 0 to 1
float32 cl_ram_usage		# RAM usage from 0 to 1


### OnboardComputerStatus
bool has_onboard_computer_status
uint8 ocs_type	 # type of onboard computer 0: Mission computer primary, 1: Mission computer backup 1, 2: Mission computer backup 2, 3: Compute node, 4-5: Compute spares, 6-9: Payload computers.

uint8[8] ocs_cpu_cores # CPU usage on the component in percent
uint8[10] ocs_cpu_combined # Combined CPU usage as the last 10 slices of 100 MS
uint8[4] ocs_gpu_cores # GPU usage on the component in percent
uint8[10] ocs_gpu_combined # Combined GPU usage as the last 10 slices of 100 MS
int8 ocs_temperature_board # [degC] Temperature of the board
int8[8] ocs_temperature_core # [degC] Temperature of the CPU core
int16[4] ocs_fan_speed # [rpm] Fan speeds
uint32 ocs_ram_usage # [MB] Amount of used RAM on the component system
uint32 ocs_ram_total # [MB] Total amount of RAM on the component system
uint32[4] ocs_storage_type # Storage type: 0: HDD, 1: SSD, 2: EMMC, 3: SD card (non-removable), 4: SD card (removable)
uint32[4] ocs_storage_usage # [MB] Amount of used storage space on the component system
uint32[4] ocs_storage_total # [MB] Total amount of storage space on the component system
uint32[6] ocs_link_type # [Kb/s] Link type: 0-9: UART, 10-19: Wired network, 20-29: Wifi, 30-39: Point-to-point proprietary, 40-49: Mesh proprietary
uint32[6] ocs_link_tx_rate # [Kb/s] Network traffic from the component system
uint32[6] ocs_link_rx_rate # [Kb/s] Network traffic to the component system
uint32[6] ocs_link_tx_max # [Kb/s] Network capacity from the component system
uint32[6] ocs_link_rx_max # [Kb/s] Network capacity to the component system


# TOPICS ant_cpu
