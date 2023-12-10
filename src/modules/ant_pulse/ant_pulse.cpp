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

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>

#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/estimator_innovations.h>
#include <uORB/topics/ant_pulse.h>


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
	// Advertise ant pulse topic.
	struct ant_pulse_s ant;
	memset(&ant, 0, sizeof(ant));
	ant.ant_id = 12345;

	orb_advert_t ant_pub = orb_advertise(ORB_ID(ant_pulse), &ant);

	// Example: run the loop synchronized to the sensor_combined topic publication
	int sensor_combined_sub = orb_subscribe(ORB_ID(sensor_combined));
	int battery_status_sub = orb_subscribe(ORB_ID(battery_status));
	int estimator_innovations_sub = orb_subscribe(ORB_ID(estimator_innovations));

	// TODO: Support multiple subscriptions -> https://docs.px4.io/main/en/modules/hello_sky.html
	px4_pollfd_struct_t fds[] = {
		{ .fd = sensor_combined_sub, .events = POLLIN },
		{ .fd = battery_status_sub, .events = POLLIN },
		{ .fd = estimator_innovations_sub, .events = POLLIN }
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
			bool updated_pulse = false;
			ant.timestamp = hrt_absolute_time();

			if (fds[0].revents & POLLIN) {
				struct sensor_combined_s sensor_combined_data;
				orb_copy(ORB_ID(sensor_combined), sensor_combined_sub, &sensor_combined_data);
				// TODO: do something with the data...
			}

			if (fds[1].revents & POLLIN) {
				struct battery_status_s battery_status_data;
				orb_copy(ORB_ID(battery_status), battery_status_sub, &battery_status_data);

				ant.has_battery_telemetry = true;
				ant.remaining = battery_status_data.remaining;
				ant.time_remaining_s = battery_status_data.time_remaining_s;
				ant.temperature = battery_status_data.temperature;
				ant.capacity = battery_status_data.capacity;
				ant.average_time_to_empty = battery_status_data.average_time_to_empty;
				ant.available_energy = battery_status_data.available_energy;
				ant.full_charge_capacity_wh = battery_status_data.full_charge_capacity_wh;
				ant.remaining_capacity_wh = battery_status_data.remaining_capacity_wh;
				ant.design_capacity = battery_status_data.design_capacity;
				ant.average_time_to_full = battery_status_data.average_time_to_full;

				updated_pulse = true;
			}

			if (fds[2].revents & POLLIN) {
				struct estimator_innovations_s estimator_innovations_data;
				orb_copy(ORB_ID(estimator_innovations), estimator_innovations_sub, &estimator_innovations_data);

				ant.has_height_telemetry = true;
				ant.rng_vpos = estimator_innovations_data.rng_vpos;
				ant.baro_vpos = estimator_innovations_data.baro_vpos;

				updated_pulse = true;
			}

			if (updated_pulse) {
				orb_publish(ORB_ID(ant_pulse), ant_pub, &ant);
			}
		}

		parameters_update();
	}

	orb_unsubscribe(sensor_combined_sub);
	orb_unsubscribe(battery_status_sub);
	orb_unsubscribe(estimator_innovations_sub);
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
