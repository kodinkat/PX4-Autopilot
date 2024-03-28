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

#include "ant_nudge.hpp"

int AntNudge::print_status()
{
	PX4_INFO("Running");
	// TODO: print additional runtime information about the state of the module

	return 0;
}

int AntNudge::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int AntNudge::task_spawn(int argc, char *argv[])
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

AntNudge *AntNudge::instantiate(int argc, char *argv[])
{
	PX4_INFO("Instantiated....!");
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

	AntNudge *instance = new AntNudge(example_param, example_flag);

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
	}

	return instance;
}

AntNudge::AntNudge(int example_param, bool example_flag)
	: ModuleParams(nullptr)
{
}

void AntNudge::run()
{
	PX4_INFO("Run() Loop....!");

	// initialize parameters
	parameters_update(true);

	int ant_system_id = 12345;
	long unsigned int last_nudge_system_id = -1;

	while (!should_exit()) {

		// Capture ant's current state.
		uORB::SubscriptionData<vehicle_status_s> vehicle_status_sub { ORB_ID(vehicle_status) };

		PX4_INFO("-- Vehicle Status Detected --");
		PX4_INFO("nav_state: %li", (long unsigned int) vehicle_status_sub.get().nav_state);
		PX4_INFO("nav_state_user_intention: %li", (long unsigned int) vehicle_status_sub.get().nav_state_user_intention);
		PX4_INFO("-----------------------------");

		// Capture ant's current local positioning.
		uORB::SubscriptionData<vehicle_local_position_s> vehicle_local_position_sub { ORB_ID(vehicle_local_position) };

		PX4_INFO("-- Vehicle Local Position Detected --");
		PX4_INFO("x: %f", (double)vehicle_local_position_sub.get().x);
		PX4_INFO("y: %f", (double)vehicle_local_position_sub.get().y);
		PX4_INFO("z: %f", (double)vehicle_local_position_sub.get().z);
		PX4_INFO("dist_bottom: %f", (double)vehicle_local_position_sub.get().dist_bottom);
		PX4_INFO("heading: %f", (double)vehicle_local_position_sub.get().heading);
		PX4_INFO("-----------------------------");

		// Capture ant's landing state.
		uORB::SubscriptionData<vehicle_land_detected_s> vehicle_land_detected_sub { ORB_ID(vehicle_land_detected) };

		PX4_INFO("-- Vehicle Land Detected --");
		PX4_INFO("freefall: %i", (bool) vehicle_land_detected_sub.get().freefall);
		PX4_INFO("ground_contact: %i", (bool) vehicle_land_detected_sub.get().ground_contact);
		PX4_INFO("maybe_landed: %i", (bool) vehicle_land_detected_sub.get().maybe_landed);
		PX4_INFO("landed: %i", (bool) vehicle_land_detected_sub.get().landed);
		PX4_INFO("in_ground_effect: %i", (bool) vehicle_land_detected_sub.get().in_ground_effect);
		PX4_INFO("in_descend: %i", (bool) vehicle_land_detected_sub.get().in_descend);
		PX4_INFO("has_low_throttle: %i", (bool) vehicle_land_detected_sub.get().has_low_throttle);
		PX4_INFO("vertical_movement: %i", (bool) vehicle_land_detected_sub.get().vertical_movement);
		PX4_INFO("horizontal_movement: %i", (bool) vehicle_land_detected_sub.get().horizontal_movement);
		PX4_INFO("close_to_ground_or_skipped_check: %i", (bool) vehicle_land_detected_sub.get().close_to_ground_or_skipped_check);
		PX4_INFO("at_rest: %i", (bool) vehicle_land_detected_sub.get().at_rest);
		PX4_INFO("-----------------------------");

		// Fetch any available incoming nudges.
		uORB::SubscriptionData<ant_nudge_s> ant_nudge_sub { ORB_ID(ant_nudge) };

		PX4_INFO("-- Ant Nudge Detected --");
		PX4_INFO("ant_system_id: %li", (long unsigned int) ant_nudge_sub.get().ant_system_id);
		PX4_INFO("nudge_system_id: %li", (long unsigned int) ant_nudge_sub.get().nudge_system_id);
		PX4_INFO("last_nudge_system_id: %li", (long unsigned int) last_nudge_system_id);
		PX4_INFO("-----------------------------");

		// Ensure detected nudge is valid and for current ant and has not been previously processed.
		if ( ( ant_nudge_sub.get().ant_system_id == ant_system_id ) && ( ant_nudge_sub.get().nudge_system_id != last_nudge_system_id ) ) {

			PX4_INFO("-- Nudge Nav Type Detected --");

			// Update last nudge system id reference.
			last_nudge_system_id = ant_nudge_sub.get().nudge_system_id;

			// Determine nudge operation to be carried out.
			bool result = false;
			int nudge_nav_type = ant_nudge_sub.get().nudge_nav_type;
			switch (nudge_nav_type)
			{
				case ant_nudge_s::NUDGE_NAV_TYPE_ARM : {
					result = arm();
				}
				break;

				case ant_nudge_s::NUDGE_NAV_TYPE_DISARM : {
					result = disarm();
				}
				break;

				case ant_nudge_s::NUDGE_NAV_TYPE_TAKEOFF : {
					result = takeoff(ant_nudge_sub.get().nnt_takeoff_altitude);
				}
				break;

				case ant_nudge_s::NUDGE_NAV_TYPE_LAND :

				case ant_nudge_s::NUDGE_NAV_TYPE_TURN_RIGHT :
				case ant_nudge_s::NUDGE_NAV_TYPE_TURN_LEFT :

				case ant_nudge_s::NUDGE_NAV_TYPE_FORWARD :
				case ant_nudge_s::NUDGE_NAV_TYPE_BACKWARD :

				case ant_nudge_s::NUDGE_NAV_TYPE_UP :
				case ant_nudge_s::NUDGE_NAV_TYPE_DOWN :

				case ant_nudge_s::NUDGE_NAV_TYPE_RIGHT :
				case ant_nudge_s::NUDGE_NAV_TYPE_LEFT : {

					// Ensure ant is currently in offboard mode.
					bool offboard_mode_enabled = true;
					if ( vehicle_status_sub.get().nav_state_user_intention != vehicle_status_s::NAVIGATION_STATE_OFFBOARD ) {
						offboard_mode_enabled = send_vehicle_command(vehicle_command_s::VEHICLE_CMD_DO_SET_MODE, 1, 6);
					}

					PX4_INFO("offboard_mode_enabled: %i", offboard_mode_enabled);

					// If offboard enabled, execute right turn.
					if ( offboard_mode_enabled ) {

						for (int i = 0; i <= 20; i++) {

							// Set offboard control mode, to that of position.
							offboard_control_mode_s ocm {};
							ocm.position = true;
							ocm.velocity = false;
							ocm.acceleration = false;
							ocm.attitude = false;
							ocm.body_rate = false;
							ocm.timestamp = hrt_absolute_time();

							uORB::Publication<offboard_control_mode_s> ocm_pub { ORB_ID(offboard_control_mode) };
							ocm_pub.publish(ocm);

							trajectory_setpoint_s ts {};
							ts.timestamp = hrt_absolute_time();

							if ( ant_nudge_s::NUDGE_NAV_TYPE_LAND == nudge_nav_type ) {

								PX4_INFO("-- Landing --");

								ts.position[0] = vehicle_local_position_sub.get().x;
								ts.position[1] = vehicle_local_position_sub.get().y;
								ts.position[2] = 0;
								ts.yaw = 0;

							} else if ( ant_nudge_s::NUDGE_NAV_TYPE_TURN_RIGHT == nudge_nav_type ) {

								PX4_INFO("-- Turning Right --");

								ts.position[0] = vehicle_local_position_sub.get().x;
								ts.position[1] = vehicle_local_position_sub.get().y;
								ts.position[2] = vehicle_local_position_sub.get().z;
								ts.yaw = vehicle_local_position_sub.get().heading + ant_nudge_sub.get().nnt_turn_right_radians;

							} else if ( ant_nudge_s::NUDGE_NAV_TYPE_TURN_LEFT == nudge_nav_type ) {

								PX4_INFO("-- Turning Left --");

								ts.position[0] = vehicle_local_position_sub.get().x;
								ts.position[1] = vehicle_local_position_sub.get().y;
								ts.position[2] = vehicle_local_position_sub.get().z;
								ts.yaw = vehicle_local_position_sub.get().heading - ant_nudge_sub.get().nnt_turn_left_radians;

							} else if ( ant_nudge_s::NUDGE_NAV_TYPE_FORWARD == nudge_nav_type ) {

								PX4_INFO("-- Forward --");

								ts.position[0] = (vehicle_local_position_sub.get().x + ant_nudge_sub.get().nnt_forward_distance) * cos(vehicle_local_position_sub.get().heading);
								ts.position[1] = (vehicle_local_position_sub.get().y + ant_nudge_sub.get().nnt_forward_distance) * sin(vehicle_local_position_sub.get().heading);
								ts.position[2] = vehicle_local_position_sub.get().z;
								ts.yaw = vehicle_local_position_sub.get().heading;

							} else if ( ant_nudge_s::NUDGE_NAV_TYPE_BACKWARD == nudge_nav_type ) {

								PX4_INFO("-- Backward --");

								ts.position[0] = (vehicle_local_position_sub.get().x - ant_nudge_sub.get().nnt_backward_distance) * cos(vehicle_local_position_sub.get().heading);
								ts.position[1] = (vehicle_local_position_sub.get().y - ant_nudge_sub.get().nnt_backward_distance) * sin(vehicle_local_position_sub.get().heading);
								ts.position[2] = vehicle_local_position_sub.get().z;
								ts.yaw = vehicle_local_position_sub.get().heading;

							} else if ( ant_nudge_s::NUDGE_NAV_TYPE_UP == nudge_nav_type ) {

								PX4_INFO("-- Up --");

								ts.position[0] = vehicle_local_position_sub.get().x;
								ts.position[1] = vehicle_local_position_sub.get().y;
								ts.position[2] = vehicle_local_position_sub.get().z - ant_nudge_sub.get().nnt_up_distance;
								ts.yaw = vehicle_local_position_sub.get().heading;

							} else if ( ant_nudge_s::NUDGE_NAV_TYPE_DOWN == nudge_nav_type ) {

								PX4_INFO("-- Down --");

								ts.position[0] = vehicle_local_position_sub.get().x;
								ts.position[1] = vehicle_local_position_sub.get().y;
								ts.position[2] = vehicle_local_position_sub.get().z + ant_nudge_sub.get().nnt_down_distance;
								ts.yaw = vehicle_local_position_sub.get().heading;

							} else if ( ant_nudge_s::NUDGE_NAV_TYPE_RIGHT == nudge_nav_type ) {

								PX4_INFO("-- Right --");

								ts.position[0] = vehicle_local_position_sub.get().x - ant_nudge_sub.get().nnt_right_distance;
								ts.position[1] = vehicle_local_position_sub.get().y;
								ts.position[2] = vehicle_local_position_sub.get().z;
								ts.yaw = vehicle_local_position_sub.get().heading;

							} else if ( ant_nudge_s::NUDGE_NAV_TYPE_LEFT == nudge_nav_type ) {

								PX4_INFO("-- Left --");

								ts.position[0] = vehicle_local_position_sub.get().x + ant_nudge_sub.get().nnt_left_distance;
								ts.position[1] = vehicle_local_position_sub.get().y;
								ts.position[2] = vehicle_local_position_sub.get().z;
								ts.yaw = vehicle_local_position_sub.get().heading;
							}

							uORB::Publication<trajectory_setpoint_s> ts_pub { ORB_ID(trajectory_setpoint) };
							result = ts_pub.publish(ts);

							px4_usleep(1000);
						}

						// Revert back to a loitering state.
						send_vehicle_command(vehicle_command_s::VEHICLE_CMD_DO_SET_MODE, 1, 4, 3);

					} else {
						// TODO: Nudge Response.
					}
				}
				break;

				default:
				break;
			}

			PX4_INFO("nudge_result: %i", result);

		}

		// Find latest command acknowledgements.
		uORB::SubscriptionData<vehicle_command_ack_s> vehicle_command_ack_sub { ORB_ID(vehicle_command_ack) };

		PX4_INFO("-- Vehicle Command Ack --");
		PX4_INFO("%li", (long unsigned int) vehicle_command_ack_sub.get().command);
		PX4_INFO("%li", (long unsigned int) vehicle_command_ack_sub.get().result);
		PX4_INFO("%li", (long unsigned int) vehicle_command_ack_sub.get().result_param1);
		PX4_INFO("%li", (long unsigned int) vehicle_command_ack_sub.get().result_param2);

		// Power nap real quick for a sec...!
		px4_sleep(1);

		parameters_update();
	}
}

void AntNudge::parameters_update(bool force)
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

int AntNudge::print_usage(const char *reason)
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

int ant_nudge_main(int argc, char *argv[])
{
	PX4_INFO("Ant Nudges Started!");
	return AntNudge::main(argc, argv);
}
