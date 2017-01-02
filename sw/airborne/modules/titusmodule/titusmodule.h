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

// Define GPS titus
#ifndef GPS_TITUS
#define GPS_TITUS 44
#endif

#include <std.h>
//#include "math/pprz_algebra_int.h"

// File logger
extern void titusmodule_init(void);
extern void titusmodule_start(void);
extern void titusmodule_periodic(void);
extern void titusmodule_stop(void);

// Guidance own=MODULE, else
// Without optitrack set to: GUIDANCE_*V/H*_GUIDANCE_H_MODE_ATTITUDE
// With optitrack set to: GUIDANCE_*V/H*_GUIDANCE_H_MODE_HOVER
#define GUIDANCE_V_MODE_MODULE_SETTING GUIDANCE_V_MODE_MODULE
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

	// imu_gyro_int32
	uint32_t gyro_stamp;
	struct Int32Rates *gyro_gyro;

	// imu_accel_int32
	uint32_t accel_stamp;
	struct Int32Vect3 *accel_accel;

	// imu_lowpassed
	uint32_t imu_stamp;
	struct Int32Rates *imu_gyro;
	struct Int32Vect3 *imu_accel;
	struct Int32Vect3 *imu_mag;

	// gps
	uint32_t gps_stamp;
	struct GpsState *gps_gps_s;

	// RC
	int rc_t;
	int rc_x;
	int rc_y;
	int rc_z;
};

#endif


