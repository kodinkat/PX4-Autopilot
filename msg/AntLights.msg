uint64 timestamp			# Time since system start (microseconds)
uint16 ant_id				# Unique ant id


### LedControl
bool has_led_control
uint8 lc_led_mask # bitmask which LED(s) to control, set to 0xff for all
uint8 lc_color # see COLOR_*
uint8 lc_mode # see MODE_*
uint8 lc_num_blinks # how many times to blink (number of on-off cycles if mode is one of MODE_BLINK_*) . Set to 0 for infinite
                 # in MODE_FLASH it is the number of cycles. Max number of blinks: 122 and max number of flash cycles: 20
uint8 lc_priority # priority: higher priority events will override current lower priority events (see MAX_PRIORITY)


# TOPICS ant_lights
