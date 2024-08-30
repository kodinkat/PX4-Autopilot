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

#pragma once

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>

// publications
#include <uORB/Publication.hpp>

// subscriptions
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionInterval.hpp>

#include <uORB/topics/ant_nudge.h>
#include <uORB/topics/ant_nudge_ack.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_command_ack.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/offboard_control_mode.h>
#include <uORB/topics/trajectory_setpoint.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_land_detected.h>

#include <uORB/topics/parameter_update.h>

using namespace time_literals;

extern "C" __EXPORT int ant_nudge_main(int argc, char *argv[]);

class AntNudge : public ModuleBase<AntNudge>, public ModuleParams
{
	public:
		AntNudge(int example_param, bool example_flag);

		virtual ~AntNudge() = default;

		/** @see ModuleBase */
		static int task_spawn(int argc, char *argv[]);

		/** @see ModuleBase */
		static AntNudge *instantiate(int argc, char *argv[]);

		/** @see ModuleBase */
		static int custom_command(int argc, char *argv[]);

		/** @see ModuleBase */
		static int print_usage(const char *reason = nullptr);

		/** @see ModuleBase::run() */
		void run() override;

		/** @see ModuleBase::print_status() */
		int print_status() override;

	private:
		/**
		 * Check for parameter changes and update them if needed.
		 * @param parameter_update_sub uorb subscription to parameter_update
		 * @param force for a parameter update
		 */
		void parameters_update(bool force = false);

		DEFINE_PARAMETERS(
			(ParamInt<px4::params::SYS_AUTOSTART>) _param_sys_autostart,   /**< example parameter */
			(ParamInt<px4::params::SYS_AUTOCONFIG>) _param_sys_autoconfig  /**< another parameter */
		)

		// Subscriptions
		uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

		static bool send_vehicle_command(const uint32_t cmd, const float param1 = NAN, const float param2 = NAN,
				 const float param3 = NAN,  const float param4 = NAN, const double param5 = static_cast<double>(NAN),
				 const double param6 = static_cast<double>(NAN), const float param7 = NAN)
		{
			vehicle_command_s vcmd{};
			vcmd.command = cmd;
			vcmd.param1 = param1;
			vcmd.param2 = param2;
			vcmd.param3 = param3;
			vcmd.param4 = param4;
			vcmd.param5 = param5;
			vcmd.param6 = param6;
			vcmd.param7 = param7;

			uORB::SubscriptionData<vehicle_status_s> vehicle_status_sub{ORB_ID(vehicle_status)};
			vcmd.source_system = vehicle_status_sub.get().system_id;
			vcmd.target_system = vehicle_status_sub.get().system_id;
			vcmd.source_component = vehicle_status_sub.get().component_id;
			vcmd.target_component = vehicle_status_sub.get().component_id;

			uORB::Publication<vehicle_command_s> vcmd_pub{ORB_ID(vehicle_command)};
			vcmd.timestamp = hrt_absolute_time();
			return vcmd_pub.publish(vcmd);
		}

		static bool wait_for_vehicle_command_reply(const uint32_t cmd,
				uORB::SubscriptionData<vehicle_command_ack_s> &vehicle_command_ack_sub)
		{
			hrt_abstime start = hrt_absolute_time();

			while (hrt_absolute_time() - start < 100_ms) {
				if (vehicle_command_ack_sub.update()) {
					if (vehicle_command_ack_sub.get().command == cmd) {
						return vehicle_command_ack_sub.get().result == vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED;
					}
				}

				px4_usleep(10000);
			}

			return false;
		}

		static bool offboard_mode()
		{
			// https://mavsdk.mavlink.io/main/en/cpp/guide/taking_off_landing.html
			// https://mavsdk.mavlink.io/main/en/cpp/guide/offboard.html
			// https://github.com/PX4/PX4-Autopilot/issues/22288
			// https://docs.px4.io/main/en/ros/ros2_offboard_control.html
			// https://github.com/PX4/px4_ros_com/blob/main/src/examples/offboard/offboard_control.cpp

			bool result = false;
			for (int i = 0; i <= 10; i++) {

				if (i == 10) {

					// Switch to offboard mode and arm
					uORB::SubscriptionData<vehicle_command_ack_s> vehicle_command_ack_sub{ ORB_ID(vehicle_command_ack) };
					send_vehicle_command(vehicle_command_s::VEHICLE_CMD_DO_SET_MODE, 1, 6);
					if (wait_for_vehicle_command_reply(vehicle_command_s::VEHICLE_CMD_DO_SET_MODE, vehicle_command_ack_sub)) {
						return send_vehicle_command(vehicle_command_s::VEHICLE_CMD_COMPONENT_ARM_DISARM,
						static_cast<float>(vehicle_command_s::ARMING_ACTION_ARM),
						0.f);
					}
				}

				// Set offboard control mode.
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

				uORB::Publication<trajectory_setpoint_s> ts_pub { ORB_ID(trajectory_setpoint) };
				ts_pub.publish(ts);

				px4_usleep(1000);
			}

			return result;
		}

		static bool takeoff(float altitude)
		{
			uORB::SubscriptionData<vehicle_command_ack_s> vehicle_command_ack_sub{ORB_ID(vehicle_command_ack)};
			send_vehicle_command(vehicle_command_s::VEHICLE_CMD_NAV_TAKEOFF,
			NAN,
			NAN,
			NAN,
			NAN,
			NAN,
			NAN,
			altitude);
			if (wait_for_vehicle_command_reply(vehicle_command_s::VEHICLE_CMD_NAV_TAKEOFF, vehicle_command_ack_sub)) {
				return send_vehicle_command(vehicle_command_s::VEHICLE_CMD_COMPONENT_ARM_DISARM,
					static_cast<float>(vehicle_command_s::ARMING_ACTION_ARM),
					0.f);
			}

			return false;
		}

		static bool land()
		{
			uORB::SubscriptionData<vehicle_command_ack_s> vehicle_command_ack_sub{ORB_ID(vehicle_command_ack)};
			send_vehicle_command(vehicle_command_s::VEHICLE_CMD_NAV_LAND);
			return wait_for_vehicle_command_reply(vehicle_command_s::VEHICLE_CMD_NAV_LAND, vehicle_command_ack_sub);
		}

		static bool arm()
		{
			return send_vehicle_command(vehicle_command_s::VEHICLE_CMD_COMPONENT_ARM_DISARM,
				static_cast<float>(vehicle_command_s::ARMING_ACTION_ARM),
				21196.f); // 21196: force arming/disarming (e.g. allow arming to override preflight checks and disarming in flight)
		}

		static bool disarm()
		{
			return send_vehicle_command(vehicle_command_s::VEHICLE_CMD_COMPONENT_ARM_DISARM,
				static_cast<float>(vehicle_command_s::ARMING_ACTION_DISARM),
				21196.f); // 21196: force arming/disarming (e.g. allow arming to override preflight checks and disarming in flight)
		}


};
