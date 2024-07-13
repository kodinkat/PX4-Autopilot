/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "ant_pulse.hpp"

int AntPulse::print_status()
{
	PX4_INFO("Running");
	// TODO: print additional runtime information about the state of the module

	return 0;
}

int AntPulse::custom_command(int argc, char *argv[])
{
	/*
	if (!is_running()) {
		print_usage("not running");
		return 1;
	}

	// additional custom commands can be handled like this:
	if (!strcmp(argv[0], "do-something")) {
		get_instance()->do_something();
		return 0;
	}
	 */

	return print_usage("unknown command");
}


int AntPulse::task_spawn(int argc, char *argv[])
{
	_task_id = px4_task_spawn_cmd("module",
				      SCHED_DEFAULT,
				      SCHED_PRIORITY_DEFAULT,
				      1024,
				      (px4_main_t)&run_trampoline,
				      (char *const *)argv);

	if (_task_id < 0) {
		_task_id = -1;
		return -errno;
	}

	return 0;
}

AntPulse *AntPulse::instantiate(int argc, char *argv[])
{
	int example_param = 0;
	bool example_flag = false;
	bool error_flag = false;

	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;

	// parse CLI arguments
	while ((ch = px4_getopt(argc, argv, "p:f", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'p':
			example_param = (int)strtol(myoptarg, nullptr, 10);
			break;

		case 'f':
			example_flag = true;
			break;

		case '?':
			error_flag = true;
			break;

		default:
			PX4_WARN("unrecognized flag");
			error_flag = true;
			break;
		}
	}

	if (error_flag) {
		return nullptr;
	}

	AntPulse *instance = new AntPulse(example_param, example_flag);

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
	}

	return instance;
}

AntPulse::AntPulse(int example_param, bool example_flag)
	: ModuleParams(nullptr)
{
}

void AntPulse::run()
{
	// Instantiate various ant topic structures to be populated accordingly, below.
	struct ant_actions_s ant_actions;
	struct ant_actuators_s ant_actuators;
	struct ant_airspeeds_s ant_airspeeds;
	struct ant_arming_s ant_arming;
	struct ant_battery_s ant_battery;
	struct ant_camera_s ant_camera;
	struct ant_cellular_s ant_cellular;
	struct ant_collisions_s ant_collisions;
	struct ant_cpu_s ant_cpu;
	struct ant_distances_s ant_distances;
	struct ant_esc_s ant_esc;
	struct ant_estimators_s ant_estimators;
	struct ant_failures_s ant_failures;
	struct ant_health_s ant_health;
	struct ant_hovering_s ant_hovering;
	struct ant_landing_s ant_landing;
	struct ant_lights_s ant_lights;
	struct ant_logs_s ant_logs;
	struct ant_power_s ant_power;
	struct ant_pulse_s ant_pulse;
	struct ant_radio_s ant_radio;
	struct ant_sensors_s ant_sensors;

	memset(&ant_actions, 0, sizeof(ant_actions));
	memset(&ant_actuators, 0, sizeof(ant_actuators));
	memset(&ant_airspeeds, 0, sizeof(ant_airspeeds));
	memset(&ant_arming, 0, sizeof(ant_arming));
	memset(&ant_battery, 0, sizeof(ant_battery));
	memset(&ant_camera, 0, sizeof(ant_camera));
	memset(&ant_cellular, 0, sizeof(ant_cellular));
	memset(&ant_collisions, 0, sizeof(ant_collisions));
	memset(&ant_cpu, 0, sizeof(ant_cpu));
	memset(&ant_distances, 0, sizeof(ant_distances));
	memset(&ant_esc, 0, sizeof(ant_esc));
	memset(&ant_estimators, 0, sizeof(ant_estimators));
	memset(&ant_failures, 0, sizeof(ant_failures));
	memset(&ant_health, 0, sizeof(ant_health));
	memset(&ant_hovering, 0, sizeof(ant_hovering));
	memset(&ant_landing, 0, sizeof(ant_landing));
	memset(&ant_lights, 0, sizeof(ant_lights));
	memset(&ant_logs, 0, sizeof(ant_logs));
	memset(&ant_power, 0, sizeof(ant_power));
	memset(&ant_pulse, 0, sizeof(ant_pulse));
	memset(&ant_radio, 0, sizeof(ant_radio));
	memset(&ant_sensors, 0, sizeof(ant_sensors));

	int ant_system_id = 12345;
	ant_actions.ant_system_id = ant_system_id;
	ant_actuators.ant_system_id = ant_system_id;
	ant_airspeeds.ant_system_id = ant_system_id;
	ant_arming.ant_system_id = ant_system_id;
	ant_battery.ant_system_id = ant_system_id;
	ant_camera.ant_system_id = ant_system_id;
	ant_cellular.ant_system_id = ant_system_id;
	ant_collisions.ant_system_id = ant_system_id;
	ant_cpu.ant_system_id = ant_system_id;
	ant_distances.ant_system_id = ant_system_id;
	ant_esc.ant_system_id = ant_system_id;
	ant_estimators.ant_system_id = ant_system_id;
	ant_failures.ant_system_id = ant_system_id;
	ant_health.ant_system_id = ant_system_id;
	ant_hovering.ant_system_id = ant_system_id;
	ant_landing.ant_system_id = ant_system_id;
	ant_lights.ant_system_id = ant_system_id;
	ant_logs.ant_system_id = ant_system_id;
	ant_power.ant_system_id = ant_system_id;
	ant_pulse.ant_system_id = ant_system_id;
	ant_radio.ant_system_id = ant_system_id;
	ant_sensors.ant_system_id = ant_system_id;

	orb_advert_t ant_actions_pub = orb_advertise(ORB_ID(ant_actions), &ant_actions);
	orb_advert_t ant_actuators_pub = orb_advertise(ORB_ID(ant_actuators), &ant_actuators);
	orb_advert_t ant_airspeeds_pub = orb_advertise(ORB_ID(ant_airspeeds), &ant_airspeeds);
	orb_advert_t ant_arming_pub = orb_advertise(ORB_ID(ant_arming), &ant_arming);
	orb_advert_t ant_battery_pub = orb_advertise(ORB_ID(ant_battery), &ant_battery);
	orb_advert_t ant_camera_pub = orb_advertise(ORB_ID(ant_camera), &ant_camera);
	orb_advert_t ant_cellular_pub = orb_advertise(ORB_ID(ant_cellular), &ant_cellular);
	orb_advert_t ant_collisions_pub = orb_advertise(ORB_ID(ant_collisions), &ant_collisions);
	orb_advert_t ant_cpu_pub = orb_advertise(ORB_ID(ant_cpu), &ant_cpu);
	orb_advert_t ant_distances_pub = orb_advertise(ORB_ID(ant_distances), &ant_distances);
	orb_advert_t ant_esc_pub = orb_advertise(ORB_ID(ant_esc), &ant_esc);
	orb_advert_t ant_estimators_pub = orb_advertise(ORB_ID(ant_estimators), &ant_estimators);
	orb_advert_t ant_failures_pub = orb_advertise(ORB_ID(ant_failures), &ant_failures);
	orb_advert_t ant_health_pub = orb_advertise(ORB_ID(ant_health), &ant_health);
	orb_advert_t ant_hovering_pub = orb_advertise(ORB_ID(ant_hovering), &ant_hovering);
	orb_advert_t ant_landing_pub = orb_advertise(ORB_ID(ant_landing), &ant_landing);
	orb_advert_t ant_lights_pub = orb_advertise(ORB_ID(ant_lights), &ant_lights);
	orb_advert_t ant_logs_pub = orb_advertise(ORB_ID(ant_logs), &ant_logs);
	orb_advert_t ant_power_pub = orb_advertise(ORB_ID(ant_power), &ant_power);
	orb_advert_t ant_pulse_pub = orb_advertise(ORB_ID(ant_pulse), &ant_pulse);
	orb_advert_t ant_radio_pub = orb_advertise(ORB_ID(ant_radio), &ant_radio);
	orb_advert_t ant_sensors_pub = orb_advertise(ORB_ID(ant_sensors), &ant_sensors);

	// Subscribe to the various orb topics of interest.
	// -> https://docs.px4.io/main/en/modules/hello_sky.html

	int action_request_sub = orb_subscribe(ORB_ID(action_request));
	int actuator_armed_sub = orb_subscribe(ORB_ID(actuator_armed));
	int actuator_motors_sub = orb_subscribe(ORB_ID(actuator_motors));
	int airspeed_sub = orb_subscribe(ORB_ID(airspeed));
	int airspeed_validated_sub = orb_subscribe(ORB_ID(airspeed_validated));
	int airspeed_wind_sub = orb_subscribe(ORB_ID(airspeed_wind));
	int arming_check_reply_sub = orb_subscribe(ORB_ID(arming_check_reply));
	int battery_status_sub = orb_subscribe(ORB_ID(battery_status));
	int camera_status_sub = orb_subscribe(ORB_ID(camera_status));
	int cellular_status_sub = orb_subscribe(ORB_ID(cellular_status));
	int collision_report_sub = orb_subscribe(ORB_ID(collision_report));
	int cpuload_sub = orb_subscribe(ORB_ID(cpuload));
	int distance_sensor_sub = orb_subscribe(ORB_ID(distance_sensor));
	int esc_report_sub = orb_subscribe(ORB_ID(esc_report));
	int esc_status_sub = orb_subscribe(ORB_ID(esc_status));
	int estimator_innovations_sub = orb_subscribe(ORB_ID(estimator_innovations));
	int estimator_status_flags_sub = orb_subscribe(ORB_ID(estimator_status_flags));
	int failsafe_flags_sub = orb_subscribe(ORB_ID(failsafe_flags));
	int failure_detector_status_sub = orb_subscribe(ORB_ID(failure_detector_status));
	int health_report_sub = orb_subscribe(ORB_ID(health_report));
	int hover_thrust_estimate_sub = orb_subscribe(ORB_ID(hover_thrust_estimate));
	int landing_gear_sub = orb_subscribe(ORB_ID(landing_gear));
	int led_control_sub = orb_subscribe(ORB_ID(led_control));
	int log_message_sub = orb_subscribe(ORB_ID(log_message));
	int mavlink_log_sub = orb_subscribe(ORB_ID(mavlink_log));
	int obstacle_distance_sub = orb_subscribe(ORB_ID(obstacle_distance));
	int onboard_computer_status_sub = orb_subscribe(ORB_ID(onboard_computer_status));
	int power_button_state_sub = orb_subscribe(ORB_ID(power_button_state));
	int power_monitor_sub = orb_subscribe(ORB_ID(power_monitor));
	int radio_status_sub = orb_subscribe(ORB_ID(radio_status));
	int sensor_accel_sub = orb_subscribe(ORB_ID(sensor_accel));
	int sensor_baro_sub = orb_subscribe(ORB_ID(sensor_baro));
	int sensor_gyro_sub = orb_subscribe(ORB_ID(sensor_gyro));
	int sensor_hygrometer_sub = orb_subscribe(ORB_ID(sensor_hygrometer));
	int sensor_mag_sub = orb_subscribe(ORB_ID(sensor_mag));
	int sensor_optical_flow_sub = orb_subscribe(ORB_ID(sensor_optical_flow));
	int system_power_sub = orb_subscribe(ORB_ID(system_power));
	int takeoff_status_sub = orb_subscribe(ORB_ID(takeoff_status));
	int vehicle_land_detected_sub = orb_subscribe(ORB_ID(vehicle_land_detected));
	int wind_sub = orb_subscribe(ORB_ID(wind));

	px4_pollfd_struct_t fds[] = {
		/*[0]*/ { .fd = action_request_sub, .events = POLLIN },
		/*[1]*/ { .fd = actuator_armed_sub, .events = POLLIN },
		/*[2]*/ { .fd = actuator_motors_sub, .events = POLLIN },
		/*[3]*/ { .fd = airspeed_sub, .events = POLLIN },
		/*[4]*/ { .fd = airspeed_validated_sub, .events = POLLIN },
		/*[5]*/ { .fd = airspeed_wind_sub, .events = POLLIN },
		/*[6]*/ { .fd = arming_check_reply_sub, .events = POLLIN },
		/*[7]*/ { .fd = battery_status_sub, .events = POLLIN },
		/*[8]*/ { .fd = camera_status_sub, .events = POLLIN },
		/*[9]*/ { .fd = cellular_status_sub, .events = POLLIN },
		/*[10]*/ { .fd = collision_report_sub, .events = POLLIN },
		/*[11]*/ { .fd = cpuload_sub, .events = POLLIN },
		/*[12]*/ { .fd = distance_sensor_sub, .events = POLLIN },
		/*[13]*/ { .fd = esc_report_sub, .events = POLLIN },
		/*[14]*/ { .fd = esc_status_sub, .events = POLLIN },
		/*[15]*/ { .fd = estimator_innovations_sub, .events = POLLIN },
		/*[16]*/ { .fd = estimator_status_flags_sub, .events = POLLIN },
		/*[17]*/ { .fd = failsafe_flags_sub, .events = POLLIN },
		/*[18]*/ { .fd = failure_detector_status_sub, .events = POLLIN },
		/*[19]*/ { .fd = health_report_sub, .events = POLLIN },
		/*[20]*/ { .fd = hover_thrust_estimate_sub, .events = POLLIN },
		/*[21]*/ { .fd = landing_gear_sub, .events = POLLIN },
		/*[22]*/ { .fd = led_control_sub, .events = POLLIN },
		/*[23]*/ { .fd = log_message_sub, .events = POLLIN },
		/*[24]*/ { .fd = mavlink_log_sub, .events = POLLIN },
		/*[25]*/ { .fd = obstacle_distance_sub, .events = POLLIN },
		/*[26]*/ { .fd = onboard_computer_status_sub, .events = POLLIN },
		/*[27]*/ { .fd = power_button_state_sub, .events = POLLIN },
		/*[28]*/ { .fd = power_monitor_sub, .events = POLLIN },
		/*[29]*/ { .fd = radio_status_sub, .events = POLLIN },
		/*[30]*/ { .fd = sensor_accel_sub, .events = POLLIN },
		/*[31]*/ { .fd = sensor_baro_sub, .events = POLLIN },
		/*[32]*/ { .fd = sensor_gyro_sub, .events = POLLIN },
		/*[33]*/ { .fd = sensor_hygrometer_sub, .events = POLLIN },
		/*[34]*/ { .fd = sensor_mag_sub, .events = POLLIN },
		/*[35]*/ { .fd = sensor_optical_flow_sub, .events = POLLIN },
		/*[36]*/ { .fd = system_power_sub, .events = POLLIN },
		/*[37]*/ { .fd = takeoff_status_sub, .events = POLLIN },
		/*[38]*/ { .fd = vehicle_land_detected_sub, .events = POLLIN },
		/*[39]*/ { .fd = wind_sub, .events = POLLIN }
	};

	// initialize parameters
	parameters_update(true);

	while (!should_exit()) {

		// wait for up to 1000ms for data
		int pret = px4_poll(fds, (sizeof(fds) / sizeof(fds[0])), 1000);

		if (pret == 0) {
			// Timeout: let the loop run anyway, don't do `continue` here

		} else if (pret < 0) {
			// this is undesirable but not much we can do
			PX4_ERR("poll error %d, %d", pret, errno);
			px4_usleep(50000);
			continue;

		} else {
			// action_request
			if (fds[0].revents & POLLIN) {
				struct action_request_s action_request_data;
				orb_copy(ORB_ID(action_request), action_request_sub, &action_request_data);

				ant_actions.timestamp = hrt_absolute_time();
				ant_actions.has_action_request = true;
				ant_actions.ar_action = action_request_data.action;
				ant_actions.ar_source = action_request_data.source;
				ant_actions.ar_mode = action_request_data.mode;
			} else {
				ant_actions.has_action_request = false;
			}

			// actuator_armed
			if (fds[1].revents & POLLIN) {
				struct actuator_armed_s actuator_armed_data;
				orb_copy(ORB_ID(actuator_armed), actuator_armed_sub, &actuator_armed_data);

				ant_actuators.timestamp = hrt_absolute_time();
				ant_actuators.has_actuator_armed = true;
				ant_actuators.aa_armed = actuator_armed_data.armed;
				ant_actuators.aa_prearmed = actuator_armed_data.prearmed;
				ant_actuators.aa_ready_to_arm = actuator_armed_data.ready_to_arm;
				ant_actuators.aa_lockdown = actuator_armed_data.lockdown;
				ant_actuators.aa_manual_lockdown = actuator_armed_data.manual_lockdown;
				ant_actuators.aa_force_failsafe = actuator_armed_data.force_failsafe;
				ant_actuators.aa_in_esc_calibration_mode = actuator_armed_data.in_esc_calibration_mode;
			} else {
				ant_actuators.has_actuator_armed = false;
			}

			// actuator_motors
			if (fds[2].revents & POLLIN) {
				struct actuator_motors_s actuator_motors_data;
				orb_copy(ORB_ID(actuator_motors), actuator_motors_sub, &actuator_motors_data);

				ant_actuators.timestamp = hrt_absolute_time();
				ant_actuators.has_actuator_motors = true;
				ant_actuators.am_reversible_flags = actuator_motors_data.reversible_flags;
				std::copy(std::begin(actuator_motors_data.control), std::end(actuator_motors_data.control), std::begin(ant_actuators.am_control));
			} else {
				ant_actuators.has_actuator_motors = false;
			}

			// airspeed
			if (fds[3].revents & POLLIN) {
				struct airspeed_s airspeed_data;
				orb_copy(ORB_ID(airspeed), airspeed_sub, &airspeed_data);

				ant_airspeeds.timestamp = hrt_absolute_time();
				ant_airspeeds.has_airspeed = true;
				ant_airspeeds.as_indicated_airspeed_m_s = airspeed_data.indicated_airspeed_m_s;
				ant_airspeeds.as_true_airspeed_m_s = airspeed_data.true_airspeed_m_s;
				ant_airspeeds.as_air_temperature_celsius = airspeed_data.air_temperature_celsius;
				ant_airspeeds.as_confidence = airspeed_data.confidence;
			} else {
				ant_airspeeds.has_airspeed = false;
			}

			// airspeed_validated
			if (fds[4].revents & POLLIN) {
				struct airspeed_validated_s airspeed_validated_data;
				orb_copy(ORB_ID(airspeed_validated), airspeed_validated_sub, &airspeed_validated_data);

				ant_airspeeds.timestamp = hrt_absolute_time();
				ant_airspeeds.has_airspeed_validated = true;
				ant_airspeeds.asv_indicated_airspeed_m_s = airspeed_validated_data.indicated_airspeed_m_s;
				ant_airspeeds.asv_calibrated_airspeed_m_s = airspeed_validated_data.calibrated_airspeed_m_s;
				ant_airspeeds.asv_true_airspeed_m_s = airspeed_validated_data.true_airspeed_m_s;
			} else {
				ant_airspeeds.has_airspeed_validated = false;
			}

			// airspeed_wind
			if (fds[5].revents & POLLIN) {
				struct airspeed_wind_s airspeed_wind_data;
				orb_copy(ORB_ID(airspeed_wind), airspeed_wind_sub, &airspeed_wind_data);

				ant_airspeeds.timestamp = hrt_absolute_time();
				ant_airspeeds.has_airspeed_wind = true;
				ant_airspeeds.asw_windspeed_north = airspeed_wind_data.windspeed_north;
				ant_airspeeds.asw_windspeed_east = airspeed_wind_data.windspeed_east;
				ant_airspeeds.asw_variance_north = airspeed_wind_data.variance_north;
				ant_airspeeds.asw_variance_east = airspeed_wind_data.variance_east;
				ant_airspeeds.asw_tas_innov = airspeed_wind_data.tas_innov;
				ant_airspeeds.asw_tas_innov_var = airspeed_wind_data.tas_innov_var;
				ant_airspeeds.asw_tas_scale_raw = airspeed_wind_data.tas_scale_raw;
				ant_airspeeds.asw_tas_scale_raw_var = airspeed_wind_data.tas_scale_raw_var;
				ant_airspeeds.asw_tas_scale_validated = airspeed_wind_data.tas_scale_validated;
				ant_airspeeds.asw_beta_innov = airspeed_wind_data.beta_innov;
				ant_airspeeds.asw_beta_innov_var = airspeed_wind_data.beta_innov_var;
				ant_airspeeds.asw_source = airspeed_wind_data.source;
			} else {
				ant_airspeeds.has_airspeed_wind = false;
			}

			// arming_check_reply
			if (fds[6].revents & POLLIN) {
				struct arming_check_reply_s arming_check_reply_data;
				orb_copy(ORB_ID(arming_check_reply), arming_check_reply_sub, &arming_check_reply_data);

				ant_arming.timestamp = hrt_absolute_time();
				ant_arming.has_arming_check_reply = true;
				ant_arming.acr_can_arm_and_run = arming_check_reply_data.can_arm_and_run;
			} else {
				ant_arming.has_arming_check_reply = false;
			}

			// battery_status
			if (fds[7].revents & POLLIN) {
				struct battery_status_s battery_status_data;
				orb_copy(ORB_ID(battery_status), battery_status_sub, &battery_status_data);

				ant_battery.timestamp = hrt_absolute_time();
				ant_battery.has_battery_status = true;
				ant_battery.bs_connected = battery_status_data.connected;
				ant_battery.bs_voltage_v = battery_status_data.voltage_v;
				//ant_battery.bs_voltage_filtered_v = battery_status_data.voltage_filtered_v;
				ant_battery.bs_current_a = battery_status_data.current_a;
				//ant_battery.bs_current_filtered_a = battery_status_data.current_filtered_a;
				ant_battery.bs_current_average_a = battery_status_data.current_average_a;
				ant_battery.bs_discharged_mah = battery_status_data.discharged_mah;
				ant_battery.bs_remaining = battery_status_data.remaining;
				ant_battery.bs_scale = battery_status_data.scale;
				ant_battery.bs_time_remaining_s = battery_status_data.time_remaining_s;
				ant_battery.bs_temperature = battery_status_data.temperature;
				ant_battery.bs_source = battery_status_data.source;
				ant_battery.bs_priority = battery_status_data.priority;
				ant_battery.bs_capacity = battery_status_data.capacity;
				ant_battery.bs_cycle_count = battery_status_data.cycle_count;
				ant_battery.bs_average_time_to_empty = battery_status_data.average_time_to_empty;
				ant_battery.bs_serial_number = battery_status_data.serial_number;
				ant_battery.bs_manufacture_date = battery_status_data.manufacture_date;
				ant_battery.bs_state_of_health = battery_status_data.state_of_health;
				ant_battery.bs_max_error = battery_status_data.max_error;
				ant_battery.bs_id = battery_status_data.id;
				ant_battery.bs_interface_error = battery_status_data.interface_error;
				std::copy(std::begin(battery_status_data.voltage_cell_v), std::end(battery_status_data.voltage_cell_v), std::begin(ant_battery.bs_voltage_cell_v));
				ant_battery.bs_max_cell_voltage_delta = battery_status_data.max_cell_voltage_delta;
				ant_battery.bs_is_powering_off = battery_status_data.is_powering_off;
				ant_battery.bs_faults = battery_status_data.faults;
				ant_battery.bs_custom_faults = battery_status_data.custom_faults;
				ant_battery.bs_warning = battery_status_data.warning;
				ant_battery.bs_mode = battery_status_data.mode;
				//ant_battery.bs_average_power = battery_status_data.average_power;
				//ant_battery.bs_available_energy = battery_status_data.available_energy;
				ant_battery.bs_full_charge_capacity_wh = battery_status_data.full_charge_capacity_wh;
				ant_battery.bs_remaining_capacity_wh = battery_status_data.remaining_capacity_wh;
				//ant_battery.bs_design_capacity = battery_status_data.design_capacity;
				//ant_battery.bs_average_time_to_full = battery_status_data.average_time_to_full;
				ant_battery.bs_over_discharge_count = battery_status_data.over_discharge_count;
				ant_battery.bs_nominal_voltage = battery_status_data.nominal_voltage;
			} else {
				ant_battery.has_battery_status = false;
			}

			// camera_status
			if (fds[8].revents & POLLIN) {
				struct camera_status_s camera_status_data;
				orb_copy(ORB_ID(camera_status), camera_status_sub, &camera_status_data);

				ant_camera.timestamp = hrt_absolute_time();
				ant_camera.has_camera_status = true;
				ant_camera.cs_active_sys_id = camera_status_data.active_sys_id;
				ant_camera.cs_active_comp_id = camera_status_data.active_comp_id;
			} else {
				ant_camera.has_camera_status = false;
			}

			// cellular_status
			if (fds[9].revents & POLLIN) {
				struct cellular_status_s cellular_status_data;
				orb_copy(ORB_ID(cellular_status), cellular_status_sub, &cellular_status_data);

				ant_cellular.timestamp = hrt_absolute_time();
				ant_cellular.has_cellular_status = true;
				ant_cellular.cs_status = cellular_status_data.status;
				ant_cellular.cs_failure_reason = cellular_status_data.failure_reason;
				ant_cellular.cs_type = cellular_status_data.type;
				ant_cellular.cs_quality = cellular_status_data.quality;
				ant_cellular.cs_mcc = cellular_status_data.mcc;
				ant_cellular.cs_mnc = cellular_status_data.mnc;
				ant_cellular.cs_lac = cellular_status_data.lac;
			} else {
				ant_cellular.has_cellular_status = false;
			}

			// collision_report
			if (fds[10].revents & POLLIN) {
				struct collision_report_s collision_report_data;
				orb_copy(ORB_ID(collision_report), collision_report_sub, &collision_report_data);

				ant_collisions.timestamp = hrt_absolute_time();
				ant_collisions.has_collision_report = true;
				ant_collisions.cr_src = collision_report_data.src;
				ant_collisions.cr_id = collision_report_data.id;
				ant_collisions.cr_action = collision_report_data.action;
				ant_collisions.cr_threat_level = collision_report_data.threat_level;
				ant_collisions.cr_time_to_minimum_delta = collision_report_data.time_to_minimum_delta;
				ant_collisions.cr_altitude_minimum_delta = collision_report_data.altitude_minimum_delta;
				ant_collisions.cr_horizontal_minimum_delta = collision_report_data.horizontal_minimum_delta;
			} else {
				ant_collisions.has_collision_report = false;
			}

			// cpuload
			if (fds[11].revents & POLLIN) {
				struct cpuload_s cpuload_data;
				orb_copy(ORB_ID(cpuload), cpuload_sub, &cpuload_data);

				ant_cpu.timestamp = hrt_absolute_time();
				ant_cpu.has_cpu_load = true;
				ant_cpu.cl_load = cpuload_data.load;
				ant_cpu.cl_ram_usage = cpuload_data.ram_usage;
			} else {
				ant_cpu.has_cpu_load = false;
			}

			// distance_sensor
			if (fds[12].revents & POLLIN) {
				struct distance_sensor_s distance_sensor_data;
				orb_copy(ORB_ID(distance_sensor), distance_sensor_sub, &distance_sensor_data);

				ant_distances.timestamp = hrt_absolute_time();
				ant_distances.has_distance_sensor = true;
				ant_distances.ds_device_id = distance_sensor_data.device_id;
				ant_distances.ds_min_distance = distance_sensor_data.min_distance;
				ant_distances.ds_max_distance = distance_sensor_data.max_distance;
				ant_distances.ds_current_distance = distance_sensor_data.current_distance;
				ant_distances.ds_variance = distance_sensor_data.variance;
				ant_distances.ds_signal_quality = distance_sensor_data.signal_quality;
				ant_distances.ds_type = distance_sensor_data.type;
				ant_distances.ds_h_fov = distance_sensor_data.h_fov;
				ant_distances.ds_v_fov = distance_sensor_data.v_fov;
				std::copy(std::begin(distance_sensor_data.q), std::end(distance_sensor_data.q), std::begin(ant_distances.ds_q));
				ant_distances.ds_orientation = distance_sensor_data.orientation;
			} else {
				ant_distances.has_distance_sensor = false;
			}

			// esc_report
			if (fds[13].revents & POLLIN) {
				struct esc_report_s esc_report_data;
				orb_copy(ORB_ID(esc_report), esc_report_sub, &esc_report_data);

				ant_esc.timestamp = hrt_absolute_time();
				ant_esc.has_esc_report = true;
				ant_esc.er_esc_errorcount = esc_report_data.esc_errorcount;
				ant_esc.er_esc_rpm = esc_report_data.esc_rpm;
				ant_esc.er_esc_voltage = esc_report_data.esc_voltage;
				ant_esc.er_esc_current = esc_report_data.esc_current;
				ant_esc.er_esc_temperature = esc_report_data.esc_temperature;
				ant_esc.er_esc_address = esc_report_data.esc_address;
				ant_esc.er_esc_cmdcount = esc_report_data.esc_cmdcount;
				ant_esc.er_esc_state = esc_report_data.esc_state;
				ant_esc.er_actuator_function = esc_report_data.actuator_function;
				ant_esc.er_failures = esc_report_data.failures;
				ant_esc.er_esc_power = esc_report_data.esc_power;
			} else {
				ant_esc.has_esc_report = false;
			}

			// esc_status
			if (fds[14].revents & POLLIN) {
				struct esc_status_s esc_status_data;
				orb_copy(ORB_ID(esc_status), esc_status_sub, &esc_status_data);

				ant_esc.timestamp = hrt_absolute_time();
				ant_esc.has_esc_status = true;
				ant_esc.es_esc_count = esc_status_data.esc_count;
				ant_esc.es_esc_connectiontype = esc_status_data.esc_connectiontype;
				ant_esc.es_esc_online_flags = esc_status_data.esc_online_flags;
				ant_esc.es_esc_armed_flags = esc_status_data.esc_armed_flags;
				//std::copy(std::begin(esc_status_data.esc), std::end(esc_status_data.esc), std::begin(ant_esc.es_esc));
			} else {
				ant_esc.has_esc_status = false;
			}

			// estimator_innovations
			if (fds[15].revents & POLLIN) {
				struct estimator_innovations_s estimator_innovations_data;
				orb_copy(ORB_ID(estimator_innovations), estimator_innovations_sub, &estimator_innovations_data);

				ant_estimators.timestamp = hrt_absolute_time();
				ant_estimators.has_estimator_innovations = true;
				ant_estimators.ei_rng_vpos = estimator_innovations_data.rng_vpos;
				ant_estimators.ei_baro_vpos = estimator_innovations_data.baro_vpos;
				std::copy(std::begin(estimator_innovations_data.flow), std::end(estimator_innovations_data.flow), std::begin(ant_estimators.ei_flow));
				//std::copy(std::begin(estimator_innovations_data.terr_flow), std::end(estimator_innovations_data.terr_flow), std::begin(ant_estimators.ei_terr_flow));
				ant_estimators.ei_heading = estimator_innovations_data.heading;
				std::copy(std::begin(estimator_innovations_data.mag_field), std::end(estimator_innovations_data.mag_field), std::begin(ant_estimators.ei_mag_field));
				std::copy(std::begin(estimator_innovations_data.gravity), std::end(estimator_innovations_data.gravity), std::begin(ant_estimators.ei_gravity));
				std::copy(std::begin(estimator_innovations_data.drag), std::end(estimator_innovations_data.drag), std::begin(ant_estimators.ei_drag));
				ant_estimators.ei_airspeed = estimator_innovations_data.airspeed;
				ant_estimators.ei_beta = estimator_innovations_data.beta;
				ant_estimators.ei_hagl = estimator_innovations_data.hagl;
				ant_estimators.ei_hagl_rate = estimator_innovations_data.hagl_rate;
			} else {
				ant_estimators.has_estimator_innovations = false;
			}

			// estimator_status_flags
			if (fds[16].revents & POLLIN) {
				struct estimator_status_flags_s estimator_status_flags_data;
				orb_copy(ORB_ID(estimator_status_flags), estimator_status_flags_sub, &estimator_status_flags_data);

				ant_estimators.timestamp = hrt_absolute_time();
				ant_estimators.has_estimator_status_flags = true;
				ant_estimators.esf_control_status_changes = estimator_status_flags_data.control_status_changes;
				ant_estimators.esf_cs_in_air = estimator_status_flags_data.cs_in_air;
				ant_estimators.esf_fault_status_changes = estimator_status_flags_data.fault_status_changes;
				ant_estimators.esf_innovation_fault_status_changes = estimator_status_flags_data.innovation_fault_status_changes;
			} else {
				ant_estimators.has_estimator_status_flags = false;
			}

			// failsafe_flags
			if (fds[17].revents & POLLIN) {
				struct failsafe_flags_s failsafe_flags_data;
				orb_copy(ORB_ID(failsafe_flags), failsafe_flags_sub, &failsafe_flags_data);

				ant_failures.timestamp = hrt_absolute_time();
				ant_failures.has_fail_safe_flags = true;
				ant_failures.fsf_battery_warning = failsafe_flags_data.battery_warning;
				ant_failures.fsf_battery_low_remaining_time = failsafe_flags_data.battery_low_remaining_time;
				ant_failures.fsf_battery_unhealthy = failsafe_flags_data.battery_unhealthy;
				ant_failures.fsf_wind_limit_exceeded = failsafe_flags_data.wind_limit_exceeded;
				ant_failures.fsf_flight_time_limit_exceeded = failsafe_flags_data.flight_time_limit_exceeded;
				ant_failures.fsf_fd_critical_failure = failsafe_flags_data.fd_critical_failure;
				ant_failures.fsf_fd_esc_arming_failure = failsafe_flags_data.fd_esc_arming_failure;
				ant_failures.fsf_fd_imbalanced_prop = failsafe_flags_data.fd_imbalanced_prop;
				ant_failures.fsf_fd_motor_failure = failsafe_flags_data.fd_motor_failure;
			} else {
				ant_failures.has_fail_safe_flags = false;
			}

			// failure_detector_status
			if (fds[18].revents & POLLIN) {
				struct failure_detector_status_s failure_detector_status_data;
				orb_copy(ORB_ID(failure_detector_status), failure_detector_status_sub, &failure_detector_status_data);

				ant_failures.timestamp = hrt_absolute_time();
				ant_failures.has_failure_detector_status = true;
				ant_failures.fds_fd_roll = failure_detector_status_data.fd_roll;
				ant_failures.fds_fd_pitch = failure_detector_status_data.fd_pitch;
				ant_failures.fds_fd_alt = failure_detector_status_data.fd_alt;
				ant_failures.fds_fd_ext = failure_detector_status_data.fd_ext;
				ant_failures.fds_fd_arm_escs = failure_detector_status_data.fd_arm_escs;
				ant_failures.fds_fd_battery = failure_detector_status_data.fd_battery;
				ant_failures.fds_fd_imbalanced_prop = failure_detector_status_data.fd_imbalanced_prop;
				ant_failures.fds_fd_motor = failure_detector_status_data.fd_motor;
				ant_failures.fds_imbalanced_prop_metric = failure_detector_status_data.imbalanced_prop_metric;
				ant_failures.fds_motor_failure_mask = failure_detector_status_data.motor_failure_mask;
			} else {
				ant_failures.has_failure_detector_status = false;
			}

			// health_report
			if (fds[19].revents & POLLIN) {
				struct health_report_s health_report_data;
				orb_copy(ORB_ID(health_report), health_report_sub, &health_report_data);

				ant_health.timestamp = hrt_absolute_time();
				ant_health.has_health_report = true;
				ant_health.hr_health_is_present_flags = health_report_data.health_is_present_flags;
				ant_health.hr_health_warning_flags = health_report_data.health_warning_flags;
				ant_health.hr_health_error_flags = health_report_data.health_error_flags;
				ant_health.hr_arming_check_warning_flags = health_report_data.arming_check_warning_flags;
				ant_health.hr_arming_check_error_flags = health_report_data.arming_check_error_flags;
			} else {
				ant_health.has_health_report = false;
			}

			// hover_thrust_estimate
			if (fds[20].revents & POLLIN) {
				struct hover_thrust_estimate_s hover_thrust_estimate_data;
				orb_copy(ORB_ID(hover_thrust_estimate), hover_thrust_estimate_sub, &hover_thrust_estimate_data);

				ant_hovering.timestamp = hrt_absolute_time();
				ant_hovering.has_hover_thrust_estimate = true;
				ant_hovering.hte_hover_thrust = hover_thrust_estimate_data.hover_thrust;
				ant_hovering.hte_hover_thrust_var = hover_thrust_estimate_data.hover_thrust_var;
				ant_hovering.hte_accel_innov = hover_thrust_estimate_data.accel_innov;
				ant_hovering.hte_accel_innov_var = hover_thrust_estimate_data.accel_innov_var;
				ant_hovering.hte_accel_innov_test_ratio = hover_thrust_estimate_data.accel_innov_test_ratio;
				ant_hovering.hte_accel_noise_var = hover_thrust_estimate_data.accel_noise_var;
				ant_hovering.hte_valid = hover_thrust_estimate_data.valid;
			} else {
				ant_hovering.has_hover_thrust_estimate = false;
			}

			// landing_gear
			if (fds[21].revents & POLLIN) {
				struct landing_gear_s landing_gear_data;
				orb_copy(ORB_ID(landing_gear), landing_gear_sub, &landing_gear_data);

				ant_landing.timestamp = hrt_absolute_time();
				ant_landing.has_landing_gear = true;
				ant_landing.lg_landing_gear = landing_gear_data.landing_gear;
			} else {
				ant_landing.has_landing_gear = false;
			}

			// led_control
			if (fds[22].revents & POLLIN) {
				struct led_control_s led_control_data;
				orb_copy(ORB_ID(led_control), led_control_sub, &led_control_data);

				ant_lights.timestamp = hrt_absolute_time();
				ant_lights.has_led_control = true;
				ant_lights.lc_led_mask = led_control_data.led_mask;
				ant_lights.lc_color = led_control_data.color;
				ant_lights.lc_mode = led_control_data.mode;
				ant_lights.lc_num_blinks = led_control_data.num_blinks;
				ant_lights.lc_priority = led_control_data.priority;
			} else {
				ant_lights.has_led_control = false;
			}

			// log_message
			if (fds[23].revents & POLLIN) {
				struct log_message_s log_message_data;
				orb_copy(ORB_ID(log_message), log_message_sub, &log_message_data);

				ant_logs.timestamp = hrt_absolute_time();
				ant_logs.has_log_message = true;
				ant_logs.lm_severity = log_message_data.severity;
				std::copy(std::begin(log_message_data.text), std::end(log_message_data.text), std::begin(ant_logs.lm_text));
			} else {
				ant_logs.has_log_message = false;
			}

			// mavlink_log
			if (fds[24].revents & POLLIN) {
				struct mavlink_log_s mavlink_log_data;
				orb_copy(ORB_ID(mavlink_log), mavlink_log_sub, &mavlink_log_data);

				ant_logs.timestamp = hrt_absolute_time();
				ant_logs.has_mavlink_log = true;
				std::copy(std::begin(mavlink_log_data.text), std::end(mavlink_log_data.text), std::begin(ant_logs.ml_text));
				ant_logs.ml_severity = mavlink_log_data.severity;
			} else {
				ant_logs.has_mavlink_log = false;
			}

			// obstacle_distance
			if (fds[25].revents & POLLIN) {
				struct obstacle_distance_s obstacle_distance_data;
				orb_copy(ORB_ID(obstacle_distance), obstacle_distance_sub, &obstacle_distance_data);

				ant_distances.timestamp = hrt_absolute_time();
				ant_distances.has_obstacle_distance = true;
				ant_distances.od_frame = obstacle_distance_data.frame;
				ant_distances.od_sensor_type = obstacle_distance_data.sensor_type;
				std::copy(std::begin(obstacle_distance_data.distances), std::end(obstacle_distance_data.distances), std::begin(ant_distances.od_distances));
				ant_distances.od_increment = obstacle_distance_data.increment;
				ant_distances.od_min_distance = obstacle_distance_data.min_distance;
				ant_distances.od_max_distance = obstacle_distance_data.max_distance;
				ant_distances.od_angle_offset = obstacle_distance_data.angle_offset;
			} else {
				ant_distances.has_obstacle_distance = false;
			}

			// onboard_computer_status
			if (fds[26].revents & POLLIN) {
				struct onboard_computer_status_s onboard_computer_status_data;
				orb_copy(ORB_ID(onboard_computer_status), onboard_computer_status_sub, &onboard_computer_status_data);

				ant_cpu.timestamp = hrt_absolute_time();
				ant_cpu.has_onboard_computer_status = true;
				ant_cpu.ocs_type = onboard_computer_status_data.type;
				std::copy(std::begin(onboard_computer_status_data.cpu_cores), std::end(onboard_computer_status_data.cpu_cores), std::begin(ant_cpu.ocs_cpu_cores));
				std::copy(std::begin(onboard_computer_status_data.cpu_combined), std::end(onboard_computer_status_data.cpu_combined), std::begin(ant_cpu.ocs_cpu_combined));
				std::copy(std::begin(onboard_computer_status_data.gpu_cores), std::end(onboard_computer_status_data.gpu_cores), std::begin(ant_cpu.ocs_gpu_cores));
				std::copy(std::begin(onboard_computer_status_data.gpu_combined), std::end(onboard_computer_status_data.gpu_combined), std::begin(ant_cpu.ocs_gpu_combined));
				ant_cpu.ocs_temperature_board = onboard_computer_status_data.temperature_board;
				std::copy(std::begin(onboard_computer_status_data.temperature_core), std::end(onboard_computer_status_data.temperature_core), std::begin(ant_cpu.ocs_temperature_core));
				std::copy(std::begin(onboard_computer_status_data.fan_speed), std::end(onboard_computer_status_data.fan_speed), std::begin(ant_cpu.ocs_fan_speed));
				ant_cpu.ocs_ram_usage = onboard_computer_status_data.ram_usage;
				ant_cpu.ocs_ram_total = onboard_computer_status_data.ram_total;
				std::copy(std::begin(onboard_computer_status_data.storage_type), std::end(onboard_computer_status_data.storage_type), std::begin(ant_cpu.ocs_storage_type));
				std::copy(std::begin(onboard_computer_status_data.storage_usage), std::end(onboard_computer_status_data.storage_usage), std::begin(ant_cpu.ocs_storage_usage));
				std::copy(std::begin(onboard_computer_status_data.storage_total), std::end(onboard_computer_status_data.storage_total), std::begin(ant_cpu.ocs_storage_total));
				std::copy(std::begin(onboard_computer_status_data.link_type), std::end(onboard_computer_status_data.link_type), std::begin(ant_cpu.ocs_link_type));
				std::copy(std::begin(onboard_computer_status_data.link_tx_rate), std::end(onboard_computer_status_data.link_tx_rate), std::begin(ant_cpu.ocs_link_tx_rate));
				std::copy(std::begin(onboard_computer_status_data.link_rx_rate), std::end(onboard_computer_status_data.link_rx_rate), std::begin(ant_cpu.ocs_link_rx_rate));
				std::copy(std::begin(onboard_computer_status_data.link_tx_max), std::end(onboard_computer_status_data.link_tx_max), std::begin(ant_cpu.ocs_link_tx_max));
				std::copy(std::begin(onboard_computer_status_data.link_rx_max), std::end(onboard_computer_status_data.link_rx_max), std::begin(ant_cpu.ocs_link_rx_max));
			} else {
				ant_cpu.has_onboard_computer_status = false;
			}

			// power_button_state
			if (fds[27].revents & POLLIN) {
				struct power_button_state_s power_button_state_data;
				orb_copy(ORB_ID(power_button_state), power_button_state_sub, &power_button_state_data);

				ant_power.timestamp = hrt_absolute_time();
				ant_power.has_power_button_state = true;
				ant_power.pbs_event = power_button_state_data.event;
			} else {
				ant_power.has_power_button_state = false;
			}

			// power_monitor
			if (fds[28].revents & POLLIN) {
				struct power_monitor_s power_monitor_data;
				orb_copy(ORB_ID(power_monitor), power_monitor_sub, &power_monitor_data);

				ant_power.timestamp = hrt_absolute_time();
				ant_power.has_power_monitor = true;
				ant_power.pm_voltage_v = power_monitor_data.voltage_v;
				ant_power.pm_current_a = power_monitor_data.current_a;
				ant_power.pm_power_w = power_monitor_data.power_w;
			} else {
				ant_power.has_power_monitor = false;
			}

			// radio_status
			if (fds[29].revents & POLLIN) {
				struct radio_status_s radio_status_data;
				orb_copy(ORB_ID(radio_status), radio_status_sub, &radio_status_data);

				ant_radio.timestamp = hrt_absolute_time();
				ant_radio.has_radio_status = true;
				ant_radio.rs_rssi = radio_status_data.rssi;
				ant_radio.rs_remote_rssi = radio_status_data.remote_rssi;
				ant_radio.rs_noise = radio_status_data.noise;
				ant_radio.rs_remote_noise = radio_status_data.remote_noise;
			} else {
				ant_radio.has_radio_status = false;
			}

			// sensor_accel
			if (fds[30].revents & POLLIN) {
				struct sensor_accel_s sensor_accel_data;
				orb_copy(ORB_ID(sensor_accel), sensor_accel_sub, &sensor_accel_data);

				ant_sensors.timestamp = hrt_absolute_time();
				ant_sensors.has_sensor_accel = true;
				ant_sensors.sa_device_id = sensor_accel_data.device_id;
				ant_sensors.sa_x = sensor_accel_data.x;
				ant_sensors.sa_y = sensor_accel_data.y;
				ant_sensors.sa_z = sensor_accel_data.z;
				ant_sensors.sa_temperature = sensor_accel_data.temperature;
			} else {
				ant_sensors.has_sensor_accel = false;
			}

			// sensor_baro
			if (fds[31].revents & POLLIN) {
				struct sensor_baro_s sensor_baro_data;
				orb_copy(ORB_ID(sensor_baro), sensor_baro_sub, &sensor_baro_data);

				ant_sensors.timestamp = hrt_absolute_time();
				ant_sensors.has_sensor_baro = true;
				ant_sensors.sb_device_id = sensor_baro_data.device_id;
				ant_sensors.sb_pressure = sensor_baro_data.pressure;
				ant_sensors.sb_temperature = sensor_baro_data.temperature;
			} else {
				ant_sensors.has_sensor_baro = false;
			}

			// sensor_gyro
			if (fds[32].revents & POLLIN) {
				struct sensor_gyro_s sensor_gyro_data;
				orb_copy(ORB_ID(sensor_gyro), sensor_gyro_sub, &sensor_gyro_data);

				ant_sensors.timestamp = hrt_absolute_time();
				ant_sensors.has_sensor_gyro = true;
				ant_sensors.sg_device_id = sensor_gyro_data.device_id;
				ant_sensors.sg_x = sensor_gyro_data.x;
				ant_sensors.sg_y = sensor_gyro_data.y;
				ant_sensors.sg_z = sensor_gyro_data.z;
				ant_sensors.sg_temperature = sensor_gyro_data.temperature;
			} else {
				ant_sensors.has_sensor_gyro = false;
			}

			// sensor_hygrometer
			if (fds[33].revents & POLLIN) {
				struct sensor_hygrometer_s sensor_hygrometer_data;
				orb_copy(ORB_ID(sensor_hygrometer), sensor_hygrometer_sub, &sensor_hygrometer_data);

				ant_sensors.timestamp = hrt_absolute_time();
				ant_sensors.has_sensor_hygrometer = true;
				ant_sensors.sh_device_id = sensor_hygrometer_data.device_id;
				ant_sensors.sh_temperature = sensor_hygrometer_data.temperature;
				ant_sensors.sh_humidity = sensor_hygrometer_data.humidity;
			} else {
				ant_sensors.has_sensor_hygrometer = false;
			}

			// sensor_mag
			if (fds[34].revents & POLLIN) {
				struct sensor_mag_s sensor_mag_data;
				orb_copy(ORB_ID(sensor_mag), sensor_mag_sub, &sensor_mag_data);

				ant_sensors.timestamp = hrt_absolute_time();
				ant_sensors.has_sensor_mag = true;
				ant_sensors.sm_device_id = sensor_mag_data.device_id;
				ant_sensors.sm_x = sensor_mag_data.x;
				ant_sensors.sm_y = sensor_mag_data.y;
				ant_sensors.sm_z = sensor_mag_data.z;
				ant_sensors.sm_temperature = sensor_mag_data.temperature;
			} else {
				ant_sensors.has_sensor_mag = false;
			}

			// sensor_optical_flow
			if (fds[35].revents & POLLIN) {
				struct sensor_optical_flow_s sensor_optical_flow_data;
				orb_copy(ORB_ID(sensor_optical_flow), sensor_optical_flow_sub, &sensor_optical_flow_data);

				ant_sensors.timestamp = hrt_absolute_time();
				ant_sensors.has_sensor_optical_flow = true;
				ant_sensors.sof_device_id = sensor_optical_flow_data.device_id;
				std::copy(std::begin(sensor_optical_flow_data.pixel_flow), std::end(sensor_optical_flow_data.pixel_flow), std::begin(ant_sensors.sof_pixel_flow));
				std::copy(std::begin(sensor_optical_flow_data.delta_angle), std::end(sensor_optical_flow_data.delta_angle), std::begin(ant_sensors.sof_delta_angle));
				ant_sensors.sof_delta_angle_available = sensor_optical_flow_data.delta_angle_available;
				ant_sensors.sof_distance_m = sensor_optical_flow_data.distance_m;
				ant_sensors.sof_distance_available = sensor_optical_flow_data.distance_available;
				ant_sensors.sof_integration_timespan_us = sensor_optical_flow_data.integration_timespan_us;
				ant_sensors.sof_quality = sensor_optical_flow_data.quality;
				ant_sensors.sof_error_count = sensor_optical_flow_data.error_count;
				ant_sensors.sof_max_flow_rate = sensor_optical_flow_data.max_flow_rate;
				ant_sensors.sof_min_ground_distance = sensor_optical_flow_data.min_ground_distance;
				ant_sensors.sof_max_ground_distance = sensor_optical_flow_data.max_ground_distance;
			} else {
				ant_sensors.has_sensor_optical_flow = false;
			}

			// system_power
			if (fds[36].revents & POLLIN) {
				struct system_power_s system_power_data;
				orb_copy(ORB_ID(system_power), system_power_sub, &system_power_data);

				ant_power.timestamp = hrt_absolute_time();
				ant_power.has_system_power = true;
				ant_power.sp_voltage5v_v = system_power_data.voltage5v_v;
				ant_power.sp_usb_connected = system_power_data.usb_connected;
				ant_power.sp_usb_valid = system_power_data.usb_valid;
			} else {
				ant_power.has_system_power = false;
			}

			// takeoff_status
			if (fds[37].revents & POLLIN) {
				struct takeoff_status_s takeoff_status_data;
				orb_copy(ORB_ID(takeoff_status), takeoff_status_sub, &takeoff_status_data);

				ant_actions.timestamp = hrt_absolute_time();
				ant_actions.has_takeoff_status = true;
				ant_actions.tos_takeoff_state = takeoff_status_data.takeoff_state;
				ant_actions.tos_tilt_limit = takeoff_status_data.tilt_limit;
			} else {
				ant_actions.has_takeoff_status = false;
			}

			// vehicle_land_detected
			if (fds[38].revents & POLLIN) {
				struct vehicle_land_detected_s vehicle_land_detected_data;
				orb_copy(ORB_ID(vehicle_land_detected), vehicle_land_detected_sub, &vehicle_land_detected_data);

				ant_landing.timestamp = hrt_absolute_time();
				ant_landing.has_vehicle_land_detected = true;
				ant_landing.vld_freefall = vehicle_land_detected_data.freefall;
				ant_landing.vld_ground_contact = vehicle_land_detected_data.ground_contact;
				ant_landing.vld_maybe_landed = vehicle_land_detected_data.maybe_landed;
				ant_landing.vld_landed = vehicle_land_detected_data.landed;
				ant_landing.vld_in_ground_effect = vehicle_land_detected_data.in_ground_effect;
				ant_landing.vld_in_descend = vehicle_land_detected_data.in_descend;
				ant_landing.vld_has_low_throttle = vehicle_land_detected_data.has_low_throttle;
				ant_landing.vld_vertical_movement = vehicle_land_detected_data.vertical_movement;
				ant_landing.vld_horizontal_movement = vehicle_land_detected_data.horizontal_movement;
				ant_landing.vld_rotational_movement = vehicle_land_detected_data.rotational_movement;
				ant_landing.vld_close_to_ground_or_skipped_check = vehicle_land_detected_data.close_to_ground_or_skipped_check;
				ant_landing.vld_at_rest = vehicle_land_detected_data.at_rest;
			} else {
				ant_landing.has_vehicle_land_detected = false;
			}

			// wind
			if (fds[39].revents & POLLIN) {
				struct wind_s wind_data;
				orb_copy(ORB_ID(wind), wind_sub, &wind_data);

				ant_airspeeds.timestamp = hrt_absolute_time();
				ant_airspeeds.has_wind = true;
				ant_airspeeds.wind_windspeed_north = wind_data.windspeed_north;
				ant_airspeeds.wind_windspeed_east = wind_data.windspeed_east;
				ant_airspeeds.wind_variance_north = wind_data.variance_north;
				ant_airspeeds.wind_variance_east = wind_data.variance_east;
				ant_airspeeds.wind_tas_innov = wind_data.tas_innov;
				ant_airspeeds.wind_tas_innov_var = wind_data.tas_innov_var;
				ant_airspeeds.wind_beta_innov = wind_data.beta_innov;
				ant_airspeeds.wind_beta_innov_var = wind_data.beta_innov_var;
			} else {
				ant_airspeeds.has_wind = false;
			}

			// Always publish ant pulse.
			orb_publish(ORB_ID(ant_pulse), ant_pulse_pub, &ant_pulse);

			// Publish any identified ant updates..
			if (ant_actions.has_action_request || ant_actions.has_takeoff_status) {
				orb_publish(ORB_ID(ant_actions), ant_actions_pub, &ant_actions);
			}

			if (ant_actuators.has_actuator_armed || ant_actuators.has_actuator_motors) {
				orb_publish(ORB_ID(ant_actuators), ant_actuators_pub, &ant_actuators);
			}

			if (ant_airspeeds.has_airspeed || ant_airspeeds.has_airspeed_validated || ant_airspeeds.has_airspeed_wind || ant_airspeeds.has_wind) {
				orb_publish(ORB_ID(ant_airspeeds), ant_airspeeds_pub, &ant_airspeeds);
			}

			if (ant_arming.has_arming_check_reply) {
				orb_publish(ORB_ID(ant_arming), ant_arming_pub, &ant_arming);
			}

			if (ant_battery.has_battery_status) {
				orb_publish(ORB_ID(ant_battery), ant_battery_pub, &ant_battery);
			}

			if (ant_camera.has_camera_status) {
				orb_publish(ORB_ID(ant_camera), ant_camera_pub, &ant_camera);
			}

			if (ant_cellular.has_cellular_status) {
				orb_publish(ORB_ID(ant_cellular), ant_cellular_pub, &ant_cellular);
			}

			if (ant_collisions.has_collision_report) {
				orb_publish(ORB_ID(ant_collisions), ant_collisions_pub, &ant_collisions);
			}

			if (ant_cpu.has_cpu_load || ant_cpu.has_onboard_computer_status) {
				orb_publish(ORB_ID(ant_cpu), ant_cpu_pub, &ant_cpu);
			}

			if (ant_distances.has_distance_sensor || ant_distances.has_obstacle_distance) {
				orb_publish(ORB_ID(ant_distances), ant_distances_pub, &ant_distances);
			}

			if (ant_esc.has_esc_report || ant_esc.has_esc_status) {
				orb_publish(ORB_ID(ant_esc), ant_esc_pub, &ant_esc);
			}

			if (ant_estimators.has_estimator_innovations || ant_estimators.has_estimator_status_flags) {
				orb_publish(ORB_ID(ant_estimators), ant_estimators_pub, &ant_estimators);
			}

			if (ant_failures.has_fail_safe_flags || ant_failures.has_failure_detector_status) {
				orb_publish(ORB_ID(ant_failures), ant_failures_pub, &ant_failures);
			}

			if (ant_health.has_health_report) {
				orb_publish(ORB_ID(ant_health), ant_health_pub, &ant_health);
			}

			if (ant_hovering.has_hover_thrust_estimate) {
				orb_publish(ORB_ID(ant_hovering), ant_hovering_pub, &ant_hovering);
			}

			if (ant_landing.has_landing_gear || ant_landing.has_vehicle_land_detected) {
				orb_publish(ORB_ID(ant_landing), ant_landing_pub, &ant_landing);
			}

			if (ant_lights.has_led_control) {
				orb_publish(ORB_ID(ant_lights), ant_lights_pub, &ant_lights);
			}

			if (ant_logs.has_log_message || ant_logs.has_mavlink_log) {
				orb_publish(ORB_ID(ant_logs), ant_logs_pub, &ant_logs);
			}

			if (ant_power.has_power_button_state || ant_power.has_power_monitor || ant_power.has_system_power) {
				orb_publish(ORB_ID(ant_power), ant_power_pub, &ant_power);
			}

			if (ant_radio.has_radio_status) {
				orb_publish(ORB_ID(ant_radio), ant_radio_pub, &ant_radio);
			}

			if (ant_sensors.has_sensor_accel || ant_sensors.has_sensor_baro || ant_sensors.has_sensor_gyro || ant_sensors.has_sensor_hygrometer || ant_sensors.has_sensor_mag || ant_sensors.has_sensor_optical_flow) {
				orb_publish(ORB_ID(ant_sensors), ant_sensors_pub, &ant_sensors);
			}
		}

		parameters_update();
	}

	// Unsubscribe from various PX4 uORB topics.
	orb_unsubscribe(action_request_sub);
	orb_unsubscribe(actuator_armed_sub);
	orb_unsubscribe(actuator_motors_sub);
	orb_unsubscribe(airspeed_sub);
	orb_unsubscribe(airspeed_validated_sub);
	orb_unsubscribe(airspeed_wind_sub);
	orb_unsubscribe(arming_check_reply_sub);
	orb_unsubscribe(battery_status_sub);
	orb_unsubscribe(camera_status_sub);
	orb_unsubscribe(cellular_status_sub);
	orb_unsubscribe(collision_report_sub);
	orb_unsubscribe(cpuload_sub);
	orb_unsubscribe(distance_sensor_sub);
	orb_unsubscribe(esc_report_sub);
	orb_unsubscribe(esc_status_sub);
	orb_unsubscribe(estimator_innovations_sub);
	orb_unsubscribe(estimator_status_flags_sub);
	orb_unsubscribe(failsafe_flags_sub);
	orb_unsubscribe(failure_detector_status_sub);
	orb_unsubscribe(health_report_sub);
	orb_unsubscribe(hover_thrust_estimate_sub);
	orb_unsubscribe(landing_gear_sub);
	orb_unsubscribe(led_control_sub);
	orb_unsubscribe(log_message_sub);
	orb_unsubscribe(mavlink_log_sub);
	orb_unsubscribe(obstacle_distance_sub);
	orb_unsubscribe(onboard_computer_status_sub);
	orb_unsubscribe(power_button_state_sub);
	orb_unsubscribe(power_monitor_sub);
	orb_unsubscribe(radio_status_sub);
	orb_unsubscribe(sensor_accel_sub);
	orb_unsubscribe(sensor_baro_sub);
	orb_unsubscribe(sensor_gyro_sub);
	orb_unsubscribe(sensor_hygrometer_sub);
	orb_unsubscribe(sensor_mag_sub);
	orb_unsubscribe(sensor_optical_flow_sub);
	orb_unsubscribe(system_power_sub);
	orb_unsubscribe(takeoff_status_sub);
	orb_unsubscribe(vehicle_land_detected_sub);
	orb_unsubscribe(wind_sub);
}

void AntPulse::parameters_update(bool force)
{
	// check for parameter updates
	if (_parameter_update_sub.updated() || force) {
		// clear update
		parameter_update_s update;
		_parameter_update_sub.copy(&update);

		// update parameters from storage
		updateParams();
	}
}

int AntPulse::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Section that describes the provided module functionality.

This is a template for a module running as a task in the background with start/stop/status functionality.

### Implementation
Section describing the high-level implementation of this module.

### Examples
CLI usage example:
$ module start -f -p 42

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("module", "template");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_FLAG('f', "Optional example flag", true);
	PRINT_MODULE_USAGE_PARAM_INT('p', 0, 0, 1000, "Optional example parameter", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int ant_pulse_main(int argc, char *argv[])
{
	return AntPulse::main(argc, argv);
}
