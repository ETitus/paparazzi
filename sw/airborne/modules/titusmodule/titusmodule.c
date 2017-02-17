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
#include "firmwares/rotorcraft/stabilization/stabilization_attitude.h"
#include "subsystems/abi.h"
#include "subsystems/datalink/telemetry.h"
#include "paparazzi.h"
#include "state.h"
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

//#ifndef TITUSMODULE_IMU_LOWPASSED_ID
//#define TITUSMODULE_IMU_LOWPASSED_ID ABI_BROADCAST
//#endif
//PRINT_CONFIG_VAR(TITUSMODULE_IMU_LOWPASSED_ID)

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
//static abi_event imu_lowpassed_ev;
static abi_event gps_ev;
static abi_event optical_flow_ev;
static abi_event velocity_estimate_ev;
//
///// Callback functions
static void titus_ctrl_agl_cb(uint8_t sender_id __attribute__((unused)), float distance);
static void titus_ctrl_imu_gyro_int32(uint8_t sender_id, uint32_t stamp, struct Int32Rates *gyro);
static void titus_ctrl_imu_accel_int32(uint8_t sender_id, uint32_t stamp, struct Int32Vect3 *accel);
//static void titus_ctrl_imu_lowpassed(uint8_t sender_id, uint32_t stamp, struct Int32Rates *gyro, struct Int32Vect3 *accel, struct Int32Vect3 *mag);
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

// Compute OptiTrack stabilization for 1/2 axes
void computeOptiTrack(bool phi, bool theta,struct Int32Eulers *opti_sp_eu);

// Functions for Optical flow control
float get_cov(float *a, float *b, int n_elements);
float get_mean_array(float *a, int n_elements);

///////////////////////////////////////// 1 axis optitrack control

/* with a pgain of 100 and a scale of 2,
 * you get an angle of 5.6 degrees for 1m pos error */
#define GH_GAIN_SCALE 2

#define MAX_POS_ERR   POS_BFP_OF_REAL(16.)
#define MAX_SPEED_ERR SPEED_BFP_OF_REAL(16.)

/*
 * internal variables
 */
struct Int32Vect2 titusmodule_pos_err;
struct Int32Vect2 titusmodule_speed_err;
struct Int32Vect2 titusmodule_ref_pos;
struct Int32Vect2 titusmodule_trim_att_integrator;

/** horizontal guidance command.
 * In north/east with #INT32_ANGLE_FRAC
 * @todo convert to real force command
 */
struct Int32Vect2  titusmodule_cmd_earth;

// Stabilizing commands
struct Int32Eulers test_sp_eu;

// Body orientations
struct Int32Eulers ned_to_body_orientation_euler;
struct Int32Quat ned_to_body_orientation_quat;

////////////////////////////////////// Optical Flow Control

// used for calculating velocity from height measurements:
#include <time.h>
long previous_time;

// number of time steps used for calculating the covariance (oscillations)
#define COV_WINDOW_SIZE 60

// How fast the gain ramps up
#define GAINRAMP 0.05

int vision_message_nr;
int previous_message_nr;

float divergence;
float divergence_vision;
float dt;
float dt2;
float cov_div;
float normalized_thrust;
uint32_t thrust;
float pstate;
float pused;
float err;

float thrust_history[COV_WINDOW_SIZE];
float divergence_history[COV_WINDOW_SIZE];
float past_divergence_history[COV_WINDOW_SIZE];
unsigned long ind_hist;

struct OpticalFlowTitus of_titusmodule;

// The struct that is logged
struct LogState TitusLog;

// sending the divergence message to the ground station:
static void send_titusmodule(struct transport_tx *trans, struct link_device *dev)
{
	pprz_msg_send_TITUSMODULE(trans, dev, AC_ID, &divergence, &divergence_vision,&cov_div,&TitusLog.distance,&err,&pused,&of_titusmodule.sum_err,&thrust);
}



//////////////////////////////////////////// Logging Module ///////////////////////////////////
void titusmodule_init(void)
{
	// Subscribe to ABI messages
	AbiBindMsgAGL(TITUSMODULE_AGL_ID, &agl_ev, titus_ctrl_agl_cb);
	AbiBindMsgIMU_GYRO_INT32(TITUSMODULE_IMU_GYRO_INT32_ID, &imu_gyro_int32_ev, titus_ctrl_imu_gyro_int32);
	AbiBindMsgIMU_ACCEL_INT32(TITUSMODULE_IMU_ACCEL_INT32_ID, &imu_accel_int32_ev, titus_ctrl_imu_accel_int32);
	//	AbiBindMsgIMU_LOWPASSED(TITUSMODULE_IMU_LOWPASSED_ID, &imu_lowpassed_ev, titus_ctrl_imu_lowpassed);
	AbiBindMsgGPS(TITUSMODULE_GPS_ID, &gps_ev, titus_ctrl_gps_cb);
	AbiBindMsgOPTICAL_FLOW(TITUSMODULE_OPTICAL_FLOW_ID, &optical_flow_ev, titus_ctrl_optical_flow_cb);
	AbiBindMsgVELOCITY_ESTIMATE(TITUSMODULE_VELOCITY_ESTIMATE_ID, &velocity_estimate_ev,titus_ctrl_velocity_cb);

	register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_TITUSMODULE, send_titusmodule);
}



void titusmodule_start(void)
{
//	file_logger_start();
}

void titusmodule_periodic(void)
{
	// Body Rates
	TitusLog.body_rates_i = stateGetBodyRates_i(); // in rad/s

//	file_logger_periodic();
}

void titusmodule_stop(void)
{
//	file_logger_stop();
}
//////////////////////////////////////////// Control Module ///////////////////////////////////


// Init V & H
void h_ctrl_module_init(void)
{
	TitusLog.rc_x = 0;
	TitusLog.rc_y = 0;
	TitusLog.rc_z = 0;
}

void v_ctrl_module_init(void)
{
	// Init V
	TitusLog.rc_t = 0;

	of_titusmodule.lp_factor = 0.95f;
	of_titusmodule.divergence_setpoint = 0.0f;
	of_titusmodule.nominal_thrust = 0.640f; //0.700f for large//  0.666f; for unknown // 0.640f with small battery
	of_titusmodule.delay_steps = 40;
	of_titusmodule.cov_set_point = -0.025f;
	of_titusmodule.sum_err = 0.0f;

	struct timespec spec;
	clock_gettime(CLOCK_REALTIME, &spec);
	previous_time = spec.tv_nsec / 1.0E6;
	//previous_time = time(NULL);

	// clear histories:
	ind_hist = 0;
	for (int i = 0; i < COV_WINDOW_SIZE; i++) {
		thrust_history[i] = 0;
		divergence_history[i] = 0;
	}

	divergence = 0;
	divergence_vision = 0.0f;
	cov_div = 0.0f;
	dt = 0.0f;
	vision_message_nr = 1;
	previous_message_nr = 0;
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
	if (!in_flight)
	{
		// Reset integrators
		stabilization_cmd[COMMAND_ROLL] = 0;
		stabilization_cmd[COMMAND_PITCH] = 0;
		stabilization_cmd[COMMAND_YAW] = 0;
	}
	else
	{

		//				test_sp_eu.phi = TitusLog.rc_x*0.2f;
		test_sp_eu.psi = ned_to_body_orientation_euler.psi;

		computeOptiTrack(1,1,&test_sp_eu);

		stabilization_attitude_set_rpy_setpoint_i(&test_sp_eu);
	}
}



// Run H
void v_ctrl_module_run(bool in_flight)
{
	if (!in_flight)
	{
		// Reset integrators
		stabilization_cmd[COMMAND_THRUST] = 0;
	}
	else
	{
		int32_t nominal_throttle = of_titusmodule.nominal_thrust * MAX_PPRZ;

		float div_factor; // factor that maps divergence in pixels as received from vision to /frame
		// ensure dt >= 0
		if (dt < 0) { dt = 0.0f; }

		// get delta time, dt, to scale the divergence measurements correctly when using "simulated" vision:
		struct timespec spec;
		clock_gettime(CLOCK_REALTIME, &spec);
		long new_time = spec.tv_nsec / 1.0E6;
		long delta_t = new_time - previous_time;
		dt2 = ((float)delta_t) / 1000.0f;
		dt += dt2;
		if (dt > 10.0f) {
			dt = 0.0f;
			return;
		}
		previous_time = new_time;

		if (vision_message_nr != previous_message_nr && dt > 1E-5 && ind_hist > 1) {
			// TODO: this div_factor depends on the subpixel-factor (automatically adapt?)
			div_factor = -1.28f; // magic number comprising field of view etc.
			float new_divergence = (divergence_vision * div_factor) / dt;

			if (fabs(new_divergence - divergence) > 0.20) {
				if (new_divergence < divergence) { new_divergence = divergence - 0.10f; }
				else { new_divergence = divergence + 0.10f; }
			}
			// low-pass filter the divergence:
			divergence = divergence * of_titusmodule.lp_factor + (new_divergence * (1.0f - of_titusmodule.lp_factor));
			previous_message_nr = vision_message_nr;
			dt = 0.0f;
		} else {
			// after re-entering the module, the divergence should be equal to the set point:
			if (ind_hist <= 1) {
				divergence = of_titusmodule.divergence_setpoint;
				for (int i = 0; i < COV_WINDOW_SIZE; i++) {
					thrust_history[i] = 0;
					divergence_history[i] = 0;
				}
				ind_hist++;
				dt = 0.0f;
				stabilization_cmd[COMMAND_THRUST] = nominal_throttle;

			}
			// else: do nothing, let dt increment
			return;
		}


		err = of_titusmodule.divergence_setpoint - divergence;


		// Time oplopende gain hier

		pused += dt2*GAINRAMP;
		dt2 = 0;

		// nominal throttle times angles?
		thrust = nominal_throttle + pused * err * MAX_PPRZ; //+ of_titusmodule.igain * of_titusmodule.sum_err * MAX_PPRZ;

		normalized_thrust = (float)(thrust / (MAX_PPRZ / 100));
		        thrust_history[ind_hist % COV_WINDOW_SIZE] = normalized_thrust;
		divergence_history[ind_hist % COV_WINDOW_SIZE] = divergence;
		int ind_past = (ind_hist % COV_WINDOW_SIZE) - of_titusmodule.delay_steps;
		while (ind_past < 0) { ind_past += COV_WINDOW_SIZE; }
		float past_divergence = divergence_history[ind_past];
		past_divergence_history[ind_hist % COV_WINDOW_SIZE] = 100.0f * past_divergence;
		ind_hist++;

		// only take covariance into account if there are enough samples in the histories:
		if (ind_hist >= COV_WINDOW_SIZE) {
			cov_div = get_cov(past_divergence_history, divergence_history, COV_WINDOW_SIZE);
		} else {
			cov_div = of_titusmodule.cov_set_point;
		}


		of_titusmodule.sum_err += err;
		stabilization_cmd[COMMAND_THRUST] = thrust;
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
	stabilization_attitude_enter();

	VECT2_COPY(titusmodule_ref_pos, *stateGetPositionNed_i());
	//	stabilization_attitude_set_failsafe_setpoint();
}

void guidance_h_module_run(bool in_flight)
{
	// Call full inner-/outerloop / horizontal-/vertical controller:
	h_ctrl_module_run(in_flight);

	stabilization_attitude_run(in_flight);
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
	// reset integrator
	of_titusmodule.sum_err = 0.0f;

	ind_hist = 0;
	pused = 1;
	cov_div = of_titusmodule.cov_set_point;
	divergence = of_titusmodule.divergence_setpoint;
	dt = 0.0f;
	dt2 = 0.0f;
	struct timespec spec;
	clock_gettime(CLOCK_REALTIME, &spec);
	previous_time = spec.tv_nsec / 1.0E6;
	vision_message_nr = 1;
	previous_message_nr = 0;
	for (int i = 0; i < COV_WINDOW_SIZE; i++) {
		thrust_history[i] = 0;
		divergence_history[i] = 0;
	}
}

void guidance_v_module_run(bool in_flight)
{
	// your vertical controller goes here
	v_ctrl_module_run(in_flight);
}

void computeOptiTrack(bool phi, bool theta,struct Int32Eulers *opti_sp_eu)
{

	bool optiVelOnly;
	optiVelOnly = 0;

	// Heading is going wrong?
	//		ned_to_body_orientation_euler = *stateGetNedToBodyEulers_i();
	//		ned_to_body_orientation_euler.psi = stateGetNedToBodyEulers_i()->psi;

	struct NedCoor_i vel_from_GPS;
	struct NedCoor_i pos_from_GPS;

	vel_from_GPS = *stateGetSpeedNed_i();
	pos_from_GPS = *stateGetPositionNed_i();

	/* maximum bank angle: default 20 deg, max 40 deg*/
	static const int32_t traj_max_bank = Min(BFP_OF_REAL(GUIDANCE_H_MAX_BANK, INT32_ANGLE_FRAC),
			BFP_OF_REAL(RadOfDeg(40), INT32_ANGLE_FRAC));
	static const int32_t total_max_bank = BFP_OF_REAL(RadOfDeg(45), INT32_ANGLE_FRAC);

	/* compute position error    */
	VECT2_DIFF(titusmodule_pos_err, titusmodule_ref_pos, pos_from_GPS);
	/* saturate it               */
	VECT2_STRIM(titusmodule_pos_err, -MAX_POS_ERR, MAX_POS_ERR);

	struct Int32Vect2 ref_speed;
	ref_speed.x = 0;
	ref_speed.y = 0;

	/* compute speed error    */
	VECT2_DIFF(titusmodule_speed_err, ref_speed, vel_from_GPS);
	/* saturate it               */
	VECT2_STRIM(titusmodule_speed_err, -MAX_SPEED_ERR, MAX_SPEED_ERR);

	if(optiVelOnly)
	{
		titusmodule_pos_err.x = 0;
		titusmodule_pos_err.y = 0;
	}

	/* run PID */
	titusmodule_cmd_earth.x =
			((GUIDANCE_H_PGAIN * titusmodule_pos_err.x) >> (INT32_POS_FRAC - GH_GAIN_SCALE)) +
			((GUIDANCE_H_DGAIN * (titusmodule_speed_err.x >> 2)) >> (INT32_SPEED_FRAC - GH_GAIN_SCALE - 2));
	titusmodule_cmd_earth.y =
			((GUIDANCE_H_PGAIN * titusmodule_pos_err.y) >> (INT32_POS_FRAC - GH_GAIN_SCALE)) +
			((GUIDANCE_H_DGAIN * (titusmodule_speed_err.y >> 2)) >> (INT32_SPEED_FRAC - GH_GAIN_SCALE - 2));

	/* trim max bank angle from PD */
	VECT2_STRIM(titusmodule_cmd_earth, -traj_max_bank, traj_max_bank);

	titusmodule_trim_att_integrator.x += (GUIDANCE_H_IGAIN * titusmodule_cmd_earth.x);
	titusmodule_trim_att_integrator.y += (GUIDANCE_H_IGAIN * titusmodule_cmd_earth.y);
	/* saturate it  */
	VECT2_STRIM(titusmodule_trim_att_integrator, -(traj_max_bank << (INT32_ANGLE_FRAC + GH_GAIN_SCALE * 2)),
			(traj_max_bank << (INT32_ANGLE_FRAC + GH_GAIN_SCALE * 2)));

	/* add it to the command */
	titusmodule_cmd_earth.x += (titusmodule_trim_att_integrator.x >> (INT32_ANGLE_FRAC + GH_GAIN_SCALE * 2));
	titusmodule_cmd_earth.y += (titusmodule_trim_att_integrator.y >> (INT32_ANGLE_FRAC + GH_GAIN_SCALE * 2));

	VECT2_STRIM(titusmodule_cmd_earth, -total_max_bank, total_max_bank);

	// Compute Angle Setpoints - Taken from Stab_att_quat
	int32_t s_psi, c_psi;
	PPRZ_ITRIG_SIN(s_psi, ned_to_body_orientation_euler.psi);
	PPRZ_ITRIG_COS(c_psi, ned_to_body_orientation_euler.psi);


	if(phi)
	{
		opti_sp_eu->phi = (-s_psi * titusmodule_cmd_earth.x + c_psi * titusmodule_cmd_earth.y) >> INT32_TRIG_FRAC;
	}
	if(theta)
	{
		opti_sp_eu->theta= -(c_psi * titusmodule_cmd_earth.x + s_psi * titusmodule_cmd_earth.y) >> INT32_TRIG_FRAC;
	}
}

/**
 * Get the mean value of an array
 * @param[out] mean The mean value
 * @param[in] *a The array
 * @param[in] n Number of elements in the array
 */
float get_mean_array(float *a, int n_elements)
{
	// determine the mean for the vector:
	float mean = 0;
	for (unsigned int i = 0; i < n_elements; i++) {
		mean += a[i];
	}
	mean /= n_elements;

	return mean;
}

/**
 * Get the covariance of two arrays
 * @param[out] cov The covariance
 * @param[in] *a The first array
 * @param[in] *b The second array
 * @param[in] n Number of elements in the arrays
 */
float get_cov(float *a, float *b, int n_elements)
{
	// Determine means for each vector:
	float mean_a = get_mean_array(a, n_elements);
	float mean_b = get_mean_array(b, n_elements);

	// Determine the covariance:
	float cov = 0;
	for (unsigned int i = 0; i < n_elements; i++) {
		cov += (a[i] - mean_a) * (b[i] - mean_b);
	}

	cov /= n_elements;

	return cov;
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
				"counter,sys_time,rc_t,rc_x,rc_y,rc_z,distance,of_stamp,flow_x,flow_y,flow_der_x,flow_der_y,of_quality,of_size_divergence,vel_stamp,vel_x,vel_y,vel_noise,gyro_stamp,gyro_p,gyro_q,gyro_r,accel_stamp,accel_x,accel_y,accel_z,body_rate_p,body_rate_q,body_rate_r,body_orien_phi,body_orien_theta,body_orien_psi,body_orien_qi,body_orien_qx,body_orien_qy,body_orien_qz,gps_stamp,gps_height,gps_heading,gps_speed,gps_ned_vel_n,gps_ned_vel_e,gps_ned_vel_d,gps_ned_vel_x,gps_ned_vel_y,gps_ecef_vel_x,gps_ecef_vel_y,gps_ecef_vel_z,gps_ecef_vel_rot_x,gps_ecef_vel_rot_y\n"
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
	fprintf(file_logger, "%d,%d,%d,%d,%d,%d,%f,%d,%d,%d,%d,%d,%f,%f,%d,%f,%f,%f,%d,%f,%f,%f,%d,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",
			counter,
			now_ts,
			// Rc messages   ,%d,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f
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
			// Gryo
			TitusLog.gyro_stamp,
			(float)TitusLog.gyro_gyro->p/12,
			(float)TitusLog.gyro_gyro->q/12,
			(float)TitusLog.gyro_gyro->r/12,
			// Accel  Check the #define INT32_ACCEL_FRAC 10 VS 1000?????????
			TitusLog.accel_stamp,
			(float)TitusLog.accel_accel->x/1000,
			(float)TitusLog.accel_accel->y/1000,
			(float)TitusLog.accel_accel->z/1000,
			// Body Rates
			(float)TitusLog.body_rates_i->p/12,
			(float)TitusLog.body_rates_i->q/12,
			(float)TitusLog.body_rates_i->r/12,
			// Body Orientation in Euler
			(float)TitusLog.ned_to_body_orientation_euler->phi/12,
			(float)TitusLog.ned_to_body_orientation_euler->theta/12,
			(float)TitusLog.ned_to_body_orientation_euler->psi/12,
			// Body Orientation in Quaternions
			(float)TitusLog.ned_to_body_orientation_quat->qi/15,
			(float)TitusLog.ned_to_body_orientation_quat->qx/15,
			(float)TitusLog.ned_to_body_orientation_quat->qy/15,
			(float)TitusLog.ned_to_body_orientation_quat->qz/15
			// GPS, check units and scaling
			//			TitusLog.gps_stamp,
			//			(float)TitusLog.gps_gps_s->hmsl/1000,
			//			(float)DegOfRad(TitusLog.gps_gps_s->course)/(1e7),
			//			(float)TitusLog.gps_gps_s->gspeed/100,
			//			(float)TitusLog.gps_gps_s->ned_vel.x/100,
			//			(float)TitusLog.gps_gps_s->ned_vel.y/100,
			//			(float)TitusLog.gps_gps_s->ned_vel.z/100,
			//			((float)TitusLog.gps_gps_s->ned_vel.x/100)*sin(TitusLog.gps_gps_s->course/(1e7))+((float)TitusLog.gps_gps_s->ned_vel.y/100)*sin( (TitusLog.gps_gps_s->course/(1e7)) - M_PI/2),
			//			((float)TitusLog.gps_gps_s->ned_vel.x/100)*cos(TitusLog.gps_gps_s->course/(1e7))+((float)TitusLog.gps_gps_s->ned_vel.y/100)*cos( (TitusLog.gps_gps_s->course/(1e7)) - M_PI/2),
			//			(float)TitusLog.gps_gps_s->ecef_vel.x/-100,
			//			(float)TitusLog.gps_gps_s->ecef_vel.y/100,
			//			(float)TitusLog.gps_gps_s->ecef_vel.z/100,
			//			((float)TitusLog.gps_gps_s->ecef_vel.x/-100)*sin(TitusLog.gps_gps_s->course/(1e7))+((float)TitusLog.gps_gps_s->ecef_vel.y/100)*sin( (TitusLog.gps_gps_s->course/(1e7)) - M_PI/2),
			//			((float)TitusLog.gps_gps_s->ecef_vel.x/-100)*cos(TitusLog.gps_gps_s->course/(1e7))+((float)TitusLog.gps_gps_s->ecef_vel.y/100)*cos( (TitusLog.gps_gps_s->course/(1e7)) - M_PI/2)
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
//static void titus_ctrl_imu_lowpassed(uint8_t sender_id, uint32_t stamp, struct Int32Rates *gyro, struct Int32Vect3 *accel, struct Int32Vect3 *mag)
//{
//	//	printf("imu updated \n");
//	TitusLog.imu_stamp = stamp;
//	TitusLog.imu_gyro = gyro;
//	TitusLog.imu_accel = accel;
//	TitusLog.imu_mag = mag;
//}
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
	printf("size_divergence: %f \n",size_divergence);
	printf("\n");
	divergence_vision = size_divergence;
	vision_message_nr++;
	if (vision_message_nr > 10) { vision_message_nr = 0; }

}
static void titus_ctrl_velocity_cb(uint8_t sender_id, uint32_t stamp, float x, float y, float z, float noise)
{
	//	printf("Velo updated \n");
	TitusLog.vel_stamp = stamp;
	TitusLog.vel_x = x;
	TitusLog.vel_y = y;
	TitusLog.vel_noise = noise;
}

