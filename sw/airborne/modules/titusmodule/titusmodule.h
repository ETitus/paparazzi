/*
 * Copyright (C) Titus
 *
 * This file is part of paparazzi
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
/**
 * @file "modules/titusmodule/titusmodule.h"
 * @author Titus
 * This module implements v and h hover based on vision
 */

#ifndef TITUSMODULE_H
#define TITUSMODULE_H


#include <std.h>
//#include "math/pprz_algebra_int.h"

// File logger
extern void titusmodule_init(void);
extern void titusmodule_start(void);
extern void titusmodule_periodic(void);
extern void titusmodule_stop(void);

// Guidance own =            MODULE
// Without optitrack set to: ATTITUDE
// With optitrack set to:    HOVER
#define GUIDANCE_V_MODE_MODULE_SETTING GUIDANCE_V_MODE_HOVER
#define GUIDANCE_H_MODE_MODULE_SETTING GUIDANCE_H_MODE_MODULE

// Implement own Horizontal loops
extern void guidance_h_module_init(void);
extern void guidance_h_module_enter(void);
extern void guidance_h_module_read_rc(void);
extern void guidance_h_module_run(bool in_flight);

// Implement own Vertical loops
extern void guidance_v_module_init(void);
extern void guidance_v_module_enter(void);
extern void guidance_v_module_run(bool in_flight);

// Settings Variables
bool ofmethode;
bool batsize;
float magicfactorX;
float magicfactorY;
bool oscphi;
bool osctheta;

struct OpticalFlowTitus {
	float lp_factor;              ///< low-pass factor in [0,1], with 0 purely using the current measurement
	float divergence_setpoint;    ///< setpoint for constant divergence approach
	float ventralflow_setpoint;    ///< setpoint for constant ventral flow approach
	float nominal_thrust;         ///< nominal thrust around which the PID-control operates
	float cov_set_point;          ///< for adaptive gain control, setpoint of the covariance (oscillations)
	int delay_steps;              ///< number of delay steps for div past

	float sum_errX;                ///< integration of the error for I-gain in X axis
	float sum_errY;                ///< integration of the error for I-gain in Y axis
	float sum_errZ;                ///< integration of the error for I-gain in Z axis

//	float cov_limit;              ///< for fixed gain control, what is the cov limit triggering the landing
//	float pgain_adaptive;         ///< P-gain for adaptive gain control
//	float igain_adaptive;         ///< I-gain for adaptive gain control
//	float dgain_adaptive;         ///< D-gain for adaptive gain control
//	float pgain;                  ///< P-gain for constant divergence control (from divergence error to thrust)
//	float igain;                  ///< I-gain for constant divergence control
//	float dgain;                  ///< D-gain for constant divergence control

};

struct LogState {

	// AGL
	float distance;

	// Velocity Estimate
	uint32_t vel_stamp;
	float vel_x;
	float vel_y;
	float vel_noise;

	// Optical Flow
	uint32_t of_stamp;
	int16_t of_flow_x;
	int16_t of_flow_y;
	int16_t of_flow_der_x;
	int16_t of_flow_der_y;
	float of_quality;
	float of_size_divergence;
	float of_divergence;

	// imu_gyro_int32
	uint32_t gyro_stamp;
	struct Int32Rates *gyro_gyro;

	// imu_accel_int32
	uint32_t accel_stamp;
	struct Int32Vect3 *accel_accel;

	// Body Rates
	struct Int32Rates *body_rates_i; // in rad/s

	// Body Orientation
	struct Int32Quat *ned_to_body_orientation_quat;
	struct Int32Eulers *ned_to_body_orientation_euler;

	// gps
	uint32_t gps_stamp;
	struct GpsState *gps_gps_s;

	// RC
	int rc_t;
	int rc_x;
	int rc_y;
	int rc_z;

	// Stabilizing commands
	struct Int32Eulers *stab_sp_eu;

	//	// imu_lowpassed
	//	uint32_t imu_stamp;
	//	struct Int32Rates *imu_gyro;
	//	struct Int32Vect3 *imu_accel;
	//	struct Int32Vect3 *imu_mag;
};

#endif


