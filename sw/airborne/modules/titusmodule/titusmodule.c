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
 * @file "modules/titusmodule/titusmodule.c"
 * @author Titus
 * This module implements v and h hover based on vision
 */

#include <stdio.h>
#include <math.h>

#include "modules/titusmodule/titusmodule.h"

//#include "state.h"
#include "firmwares/rotorcraft/stabilization.h"
#include "subsystems/abi.h"
#include "paparazzi.h"
#include "subsystems/radio_control.h"
//#include "generated/airframe.h"
//#include "firmwares/rotorcraft/autopilot.h"
//#include "subsystems/navigation/common_flight_plan.h"

// Set the default log path
#ifndef FILE_LOGGER_PATH
#define FILE_LOGGER_PATH /data/video
#endif

// The file pointer
static FILE *file_logger = NULL;

// Define ABI IDs
#ifndef TITUSMODULE_AGL_ID
#define TITUSMODULE_AGL_ID ABI_BROADCAST
#endif
PRINT_CONFIG_VAR(TITUSMODULE_AGL_ID)
//
#ifndef TITUSMODULE_IMU_GYRO_INT32_ID
#define TITUSMODULE_IMU_GYRO_INT32_ID ABI_BROADCAST
#endif
PRINT_CONFIG_VAR(TITUSMODULE_IMU_GYRO_INT32_ID)

#ifndef TITUSMODULE_IMU_ACCEL_INT32_ID
#define TITUSMODULE_IMU_ACCEL_INT32_ID ABI_BROADCAST
#endif
PRINT_CONFIG_VAR(TITUSMODULE_IMU_ACCEL_INT32_ID)

#ifndef TITUSMODULE_IMU_LOWPASSED_ID
#define TITUSMODULE_IMU_LOWPASSED_ID ABI_BROADCAST
#endif
PRINT_CONFIG_VAR(TITUSMODULE_IMU_LOWPASSED_ID)

#ifndef TITUSMODULE_GPS_ID
#define TITUSMODULE_GPS_ID ABI_BROADCAST
#endif
PRINT_CONFIG_VAR(TITUSMODULE_GPS_ID)

#ifndef TITUSMODULE_OPTICAL_FLOW_ID
#define TITUSMODULE_OPTICAL_FLOW_ID ABI_BROADCAST
#endif
PRINT_CONFIG_VAR(TITUSMODULE_OPTICAL_FLOW_ID)

#ifndef TITUSMODULE_VELOCITY_ESTIMATE_ID
#define TITUSMODULE_VELOCITY_ESTIMATE_ID ABI_BROADCAST
#endif
PRINT_CONFIG_VAR(TITUSMODULE_VELOCITY_ESTIMATE_ID)


// The ABI events
static abi_event agl_ev;
static abi_event imu_gyro_int32_ev;
static abi_event imu_accel_int32_ev;
static abi_event imu_lowpassed_ev;
static abi_event gps_ev;
static abi_event optical_flow_ev;
static abi_event velocity_estimate_ev;
//
///// Callback functions
static void titus_ctrl_agl_cb(uint8_t sender_id __attribute__((unused)), float distance);
static void titus_ctrl_imu_gyro_int32(uint8_t sender_id, uint32_t stamp, struct Int32Rates *gyro);
static void titus_ctrl_imu_accel_int32(uint8_t sender_id, uint32_t stamp, struct Int32Vect3 *accel);
static void titus_ctrl_imu_lowpassed(uint8_t sender_id, uint32_t stamp, struct Int32Rates *gyro, struct Int32Vect3 *accel, struct Int32Vect3 *mag);
static void titus_ctrl_gps_cb(uint8_t sender_id __attribute__((unused)), uint32_t stamp, struct GpsState *gps_s);
static void titus_ctrl_optical_flow_cb(uint8_t sender_id __attribute__((unused)), uint32_t stamp, int16_t flow_x, int16_t flow_y, int16_t flow_der_x, int16_t flow_der_y, float quality, float size_divergence, float dist);
static void titus_ctrl_velocity_cb(uint8_t sender_id __attribute__((unused)), uint32_t stamp, float x, float y, float z, float noise);

// Run & init functions
void h_ctrl_module_init(void);
void h_ctrl_module_run(bool in_flight);
void v_ctrl_module_init(void);
void v_ctrl_module_run(bool in_flight);

void file_logger_start(void);
void file_logger_periodic(void);
void file_logger_stop(void);

// The struct that is logged
struct LogState TitusLog;


void titusmodule_init(void)
{
	// Subscribe to ABI messages
	AbiBindMsgAGL(TITUSMODULE_AGL_ID, &agl_ev, titus_ctrl_agl_cb);
	AbiBindMsgIMU_GYRO_INT32(TITUSMODULE_IMU_GYRO_INT32_ID, &imu_gyro_int32_ev, titus_ctrl_imu_gyro_int32);
	AbiBindMsgIMU_ACCEL_INT32(TITUSMODULE_IMU_ACCEL_INT32_ID, &imu_accel_int32_ev, titus_ctrl_imu_accel_int32);
	AbiBindMsgIMU_LOWPASSED(TITUSMODULE_IMU_LOWPASSED_ID, &imu_lowpassed_ev, titus_ctrl_imu_lowpassed);
	AbiBindMsgGPS(TITUSMODULE_GPS_ID, &gps_ev, titus_ctrl_gps_cb);
	AbiBindMsgOPTICAL_FLOW(TITUSMODULE_OPTICAL_FLOW_ID, &optical_flow_ev, titus_ctrl_optical_flow_cb);
	AbiBindMsgVELOCITY_ESTIMATE(TITUSMODULE_VELOCITY_ESTIMATE_ID, &velocity_estimate_ev,titus_ctrl_velocity_cb);
}



void titusmodule_start(void)
{
	file_logger_start();
}

void titusmodule_periodic(void)
{
	file_logger_periodic();
}


void titusmodule_stop(void)
{
	file_logger_stop();
}
//////////////////////////////////////////// Control Module ///////////////////////////////////


// Init V & H
void h_ctrl_module_init(void)
{
	TitusLog.rc_t = 0;
	TitusLog.rc_x = 0;
	TitusLog.rc_y = 0;
	TitusLog.rc_z = 0;
}

void v_ctrl_module_init(void)
{
	// Init V
}

// Read H RC
void guidance_h_module_read_rc(void)
{
	//	printf("h read RC");
	TitusLog.rc_t = radio_control.values[RADIO_THROTTLE];
	TitusLog.rc_x = radio_control.values[RADIO_ROLL];
	TitusLog.rc_y = radio_control.values[RADIO_PITCH];
	TitusLog.rc_z = radio_control.values[RADIO_YAW];
}

// Run H
void h_ctrl_module_run(bool in_flight)
{
	if (!in_flight) {
		// Reset integrators
		stabilization_cmd[COMMAND_ROLL] = 0;
		stabilization_cmd[COMMAND_PITCH] = 0;
		stabilization_cmd[COMMAND_YAW] = 0;
		stabilization_cmd[COMMAND_THRUST] = 0;
	} else {
		stabilization_cmd[COMMAND_ROLL] = TitusLog.rc_x;
		stabilization_cmd[COMMAND_PITCH] = TitusLog.rc_y;
		stabilization_cmd[COMMAND_YAW] = TitusLog.rc_z;
	}
}

// Run H
void v_ctrl_module_run(bool in_flight)
{
	if (!in_flight) {
		// Reset integrators
		stabilization_cmd[COMMAND_ROLL] = 0;
		stabilization_cmd[COMMAND_PITCH] = 0;
		stabilization_cmd[COMMAND_YAW] = 0;
		stabilization_cmd[COMMAND_THRUST] = 0;
	} else {
		stabilization_cmd[COMMAND_THRUST] = TitusLog.rc_t;
	}
}

////////////////////////////////////////////////////////////////////
// Call our controllers
// Implement own Horizontal loops
void guidance_h_module_init(void)
{
	h_ctrl_module_init();
}

void guidance_h_module_enter(void)
{
	// run Enter module code here
}

void guidance_h_module_run(bool in_flight)
{
	// Call full inner-/outerloop / horizontal-/vertical controller:
	h_ctrl_module_run(in_flight);
}



// Implement own Vertical loops
void guidance_v_module_init(void)
{
	// initialization of your custom vertical controller goes here
	v_ctrl_module_init();
}

void guidance_v_module_enter(void)
{
	// your code that should be executed when entering this vertical mode goes here
}

void guidance_v_module_run(bool in_flight)
{
	// your vertical controller goes here
	v_ctrl_module_run(in_flight);
}


void file_logger_start(void)
{
	uint32_t counter = 0;
	char filename[512];

	// Check for available files
	sprintf(filename, "%s/%05d.csv", STRINGIFY(FILE_LOGGER_PATH), counter);
	while ((file_logger = fopen(filename, "r"))) {
		fclose(file_logger);

		counter++;
		sprintf(filename, "%s/%05d.csv", STRINGIFY(FILE_LOGGER_PATH), counter);
	}

	file_logger = fopen(filename, "w");

	if (file_logger != NULL) {
		fprintf(
				file_logger,
				"counter,sys_time,rc_t,rc_x,rc_y,rc_z,distance,of_stamp,flow_x,flow_y,flow_der_x,flow_der_y,of_quality,of_size_divergence,vel_stamp,vel_x,vel_y,vel_noise,gyro_stamp,gyro_p,gyro_q,gyro_r,accel_stamp,accel_x,accel_y,accel_z,imu_stamp,imu_acc_x,imu_acc_y,imu_acc_z,imu_gyro_p,imu_gyro_q,imu_gyro_r,imu_mag_x,imu_mag_y,imu_mag_z,gps_stamp,gps_height,gps_heading,gps_speed,gps_ned_vel_n,gps_ned_vel_e,gps_ned_vel_d,gps_ned_vel_x,gps_ned_vel_y,gps_ecef_vel_x,gps_ecef_vel_y,gps_ecef_vel_z,gps_ecef_vel_rot_x,gps_ecef_vel_rot_y\n"
		);
	}
}

void file_logger_periodic(void)
{
	if (file_logger == NULL) {
		return;
	}
	static uint32_t counter;
	uint32_t now_ts = get_sys_time_usec();
	fprintf(file_logger, "%d,%d,%d,%d,%d,%d,%f,%d,%d,%d,%d,%d,%f,%f,%d,%f,%f,%f,%d,%f,%f,%f,%d,%f,%f,%f,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",
			counter,
			now_ts,
			// Rc messages
			TitusLog.rc_t,
			TitusLog.rc_x,
			TitusLog.rc_y,
			TitusLog.rc_z,
			// Distance
			TitusLog.distance,
			// Optical Flow
			TitusLog.of_stamp,
			TitusLog.of_flow_x,
			TitusLog.of_flow_y,
			TitusLog.of_flow_der_x,
			TitusLog.of_flow_der_y,
			TitusLog.of_quality,
			TitusLog.of_size_divergence,
			// Velocity estimation
			TitusLog.vel_stamp,
			TitusLog.vel_x,
			TitusLog.vel_y,
			TitusLog.vel_noise,
			TitusLog.gyro_stamp,
			// Check the #define INT32_RATE_FRAC 12
			(float)TitusLog.gyro_gyro->p/12,
			(float)TitusLog.gyro_gyro->q/12,
			(float)TitusLog.gyro_gyro->r/12,
			TitusLog.accel_stamp,
			// Check the #define INT32_ACCEL_FRAC 10 VS 1000?????????
			(float)TitusLog.accel_accel->x/1000,
			(float)TitusLog.accel_accel->y/1000,
			(float)TitusLog.accel_accel->z/1000,
			// Doesnt log often? Check Scaling!
			TitusLog.imu_stamp,
			TitusLog.imu_accel->x/1000,
			TitusLog.imu_accel->y/1000,
			TitusLog.imu_accel->z/1000,
			TitusLog.imu_gyro->p/12,
			TitusLog.imu_gyro->q/12,
			TitusLog.imu_gyro->r/12,
			TitusLog.imu_mag->x/1000,
			TitusLog.imu_mag->y/1000,
			TitusLog.imu_mag->z/1000,
			// GPS, check units and scaling
			TitusLog.gps_stamp,
			(float)TitusLog.gps_gps_s->hmsl/1000,
			(float)DegOfRad(TitusLog.gps_gps_s->course)/(1e7),
			(float)TitusLog.gps_gps_s->gspeed/100,
			(float)TitusLog.gps_gps_s->ned_vel.x/100,
			(float)TitusLog.gps_gps_s->ned_vel.y/100,
			(float)TitusLog.gps_gps_s->ned_vel.z/100,
			((float)TitusLog.gps_gps_s->ned_vel.x/100)*sin(TitusLog.gps_gps_s->course/(1e7))+((float)TitusLog.gps_gps_s->ned_vel.y/100)*sin( (TitusLog.gps_gps_s->course/(1e7)) - M_PI/2),
			((float)TitusLog.gps_gps_s->ned_vel.x/100)*cos(TitusLog.gps_gps_s->course/(1e7))+((float)TitusLog.gps_gps_s->ned_vel.y/100)*cos( (TitusLog.gps_gps_s->course/(1e7)) - M_PI/2),
			(float)TitusLog.gps_gps_s->ecef_vel.x/-100,
			(float)TitusLog.gps_gps_s->ecef_vel.y/100,
			(float)TitusLog.gps_gps_s->ecef_vel.z/100,
			((float)TitusLog.gps_gps_s->ecef_vel.x/-100)*sin(TitusLog.gps_gps_s->course/(1e7))+((float)TitusLog.gps_gps_s->ecef_vel.y/100)*sin( (TitusLog.gps_gps_s->course/(1e7)) - M_PI/2),
			((float)TitusLog.gps_gps_s->ecef_vel.x/-100)*cos(TitusLog.gps_gps_s->course/(1e7))+((float)TitusLog.gps_gps_s->ecef_vel.y/100)*cos( (TitusLog.gps_gps_s->course/(1e7)) - M_PI/2)
			);
	counter++;
}

/** Stop the logger an nicely close the file */
void file_logger_stop(void)
{
	if (file_logger != NULL) {
		fclose(file_logger);
		file_logger = NULL;
	}
}

// Read sensors
static void titus_ctrl_agl_cb(uint8_t sender_id, float distance)
{
	//	printf("distance updated \n");
	TitusLog.distance = distance;

}
static void titus_ctrl_imu_gyro_int32(uint8_t sender_id, uint32_t stamp, struct Int32Rates *gyro)
{
	//	printf("gyro updated\n");
	TitusLog.gyro_stamp = stamp;
	TitusLog.gyro_gyro = gyro;
}
static void titus_ctrl_imu_accel_int32(uint8_t sender_id, uint32_t stamp, struct Int32Vect3 *accel)
{
	//	printf("accel updated \n");
	TitusLog.accel_stamp = stamp;
	TitusLog.accel_accel = accel;
}
static void titus_ctrl_imu_lowpassed(uint8_t sender_id, uint32_t stamp, struct Int32Rates *gyro, struct Int32Vect3 *accel, struct Int32Vect3 *mag)
{
	//	printf("imu updated \n");
	TitusLog.imu_stamp = stamp;
	TitusLog.imu_gyro = gyro;
	TitusLog.imu_accel = accel;
	TitusLog.imu_mag = mag;
}
static void titus_ctrl_gps_cb(uint8_t sender_id, uint32_t stamp, struct GpsState *gps_s)
{
	//		printf("gps updated \n");
	TitusLog.gps_stamp = stamp;
	TitusLog.gps_gps_s = gps_s;
}
static void titus_ctrl_optical_flow_cb(uint8_t sender_id, uint32_t stamp, int16_t flow_x, int16_t flow_y, int16_t flow_der_x, int16_t flow_der_y, float quality, float size_divergence, float dist)
{
	//	printf("OF updated \n");
	TitusLog.of_stamp = stamp;
	TitusLog.of_flow_x = flow_x;
	TitusLog.of_flow_y = flow_y;
	TitusLog.of_flow_der_x = flow_der_x;
	TitusLog.of_flow_der_y = flow_der_y;
	TitusLog.of_quality = quality;
	TitusLog.of_size_divergence = size_divergence;
}
static void titus_ctrl_velocity_cb(uint8_t sender_id, uint32_t stamp, float x, float y, float z, float noise)
{
	//	printf("Velo updated \n");
	TitusLog.vel_stamp = stamp;
	TitusLog.vel_x = x;
	TitusLog.vel_y = y;
	TitusLog.vel_noise = noise;
}

