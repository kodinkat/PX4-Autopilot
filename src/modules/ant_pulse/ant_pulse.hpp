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

#include <uORB/SubscriptionInterval.hpp>

#include <uORB/topics/ant_actions.h>
#include <uORB/topics/ant_actuators.h>
#include <uORB/topics/ant_airspeeds.h>
#include <uORB/topics/ant_arming.h>
#include <uORB/topics/ant_battery.h>
#include <uORB/topics/ant_camera.h>
#include <uORB/topics/ant_cellular.h>
#include <uORB/topics/ant_collisions.h>
#include <uORB/topics/ant_cpu.h>
#include <uORB/topics/ant_distances.h>
#include <uORB/topics/ant_esc.h>
#include <uORB/topics/ant_estimators.h>
#include <uORB/topics/ant_failures.h>
#include <uORB/topics/ant_health.h>
#include <uORB/topics/ant_hovering.h>
#include <uORB/topics/ant_landing.h>
#include <uORB/topics/ant_lights.h>
#include <uORB/topics/ant_logs.h>
#include <uORB/topics/ant_power.h>
#include <uORB/topics/ant_pulse.h>
#include <uORB/topics/ant_radio.h>
#include <uORB/topics/ant_sensors.h>
#include <uORB/topics/ant_vehicle_local_position.h>

#include <uORB/topics/parameter_update.h>
#include <uORB/topics/action_request.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/actuator_motors.h>
#include <uORB/topics/airspeed.h>
#include <uORB/topics/airspeed_validated.h>
#include <uORB/topics/airspeed_wind.h>
#include <uORB/topics/arming_check_reply.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/camera_status.h>
#include <uORB/topics/cellular_status.h>
#include <uORB/topics/collision_report.h>
#include <uORB/topics/cpuload.h>
#include <uORB/topics/distance_sensor.h>
#include <uORB/topics/esc_report.h>
#include <uORB/topics/esc_status.h>
#include <uORB/topics/estimator_innovations.h>
#include <uORB/topics/estimator_status_flags.h>
#include <uORB/topics/failsafe_flags.h>
#include <uORB/topics/failure_detector_status.h>
#include <uORB/topics/health_report.h>
#include <uORB/topics/hover_thrust_estimate.h>
#include <uORB/topics/landing_gear.h>
#include <uORB/topics/led_control.h>
#include <uORB/topics/log_message.h>
#include <uORB/topics/mavlink_log.h>
#include <uORB/topics/obstacle_distance.h>
#include <uORB/topics/onboard_computer_status.h>
#include <uORB/topics/power_button_state.h>
#include <uORB/topics/power_monitor.h>
#include <uORB/topics/radio_status.h>
#include <uORB/topics/sensor_accel.h>
#include <uORB/topics/sensor_baro.h>
#include <uORB/topics/sensor_gyro.h>
#include <uORB/topics/sensor_hygrometer.h>
#include <uORB/topics/sensor_mag.h>
#include <uORB/topics/sensor_optical_flow.h>
#include <uORB/topics/system_power.h>
#include <uORB/topics/takeoff_status.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/wind.h>
#include <uORB/topics/vehicle_local_position.h>

#include <algorithm>
#include <iterator>

using namespace time_literals;

extern "C" __EXPORT int ant_pulse_main(int argc, char *argv[]);

class AntPulse : public ModuleBase<AntPulse>, public ModuleParams
{
public:
	AntPulse(int example_param, bool example_flag);

	virtual ~AntPulse() = default;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static AntPulse *instantiate(int argc, char *argv[]);

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

};
