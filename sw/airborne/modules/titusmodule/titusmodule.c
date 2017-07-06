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
#include "subsystems/electrical.h"

// debug
#include <sys/time.h>

#include "modules/computer_vision/lib/encoding/jpeg.h"
#include "modules/computer_vision/opticflow/opticflow_calculator.h"


#ifndef TITUSMODULE_OPTICAL_FLOW_ID
#define TITUSMODULE_OPTICAL_FLOW_ID ABI_BROADCAST
#endif
PRINT_CONFIG_VAR(TITUSMODULE_OPTICAL_FLOW_ID)

// The ABI event
static abi_event optical_flow_ev;

static uint32_t timeval_diff(struct timeval *starttime, struct timeval *finishtime);

///// Callback function
static void titus_ctrl_optical_flow_cb(uint8_t sender_id __attribute__((unused)), uint32_t stamp, int16_t flow_x, int16_t flow_y, int16_t flow_der_x, int16_t flow_der_y, float quality, float size_div,float regular_div, float dist);

// Run & init functions
void h_ctrl_module_init(void);
void h_ctrl_module_run(bool in_flight);
void v_ctrl_module_init(void);
void v_ctrl_module_run(bool in_flight);

// Compute OptiTrack stabilization for 1/2 axes
void computeOptiTrack(bool phi, bool theta,struct Int32Eulers *opti_sp_eu);

// Functions for Optical flow control
float get_cov(float *a, float *b, int n_elements);
float get_mean_array(float *a, int n_elements);

///////////////////////////////////////// 1 axis optitrack control

/* with a pgain of 100 and a scale of 2,
 * you get an angle of 5.6 degrees for 1m pos error */
#define GH_GAIN_SCALE 2

#define MAX_POS_ERR POS_BFP_OF_REAL(16.)
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

// number of time steps used for calculating the covariance (oscillations)
#define COV_WINDOW_SIZE 60

int vision_message_nr;
int previous_message_nrXY;
int previous_message_nrZ;



// FF Takeoff  //////////////////////////////////////////////////
bool inFFtakeoff;
bool setFFtime;
struct timeval FF_starttime;



// in Z  //////////////////////////////////////////////////
long previous_timeZ;

// How fast the gain ramps up
#define GAINRAMPZ 0.5


float divergence;
float size_divergence;
float regular_divergence;
float divergence_vision;
float dtZ;
float dtZ2;

float cov_divZ;
//float normalized_thrust;
uint32_t thrust;
float pusedZ;
float errZ;

bool oscillatingZ;
bool algoZ;

//float thrust_history[COV_WINDOW_SIZE];
float divergence_history[COV_WINDOW_SIZE];
float past_divergence_history[COV_WINDOW_SIZE];
unsigned long ind_histZ;

float heightEstimate;

float OptiThrust;
// in XY  //////////////////////////////////////////////////
long previous_timeXY;

float dtXY;
float dtXY2;

unsigned long ind_histXY;
float ventralX_history[COV_WINDOW_SIZE];
float past_ventralX_history[COV_WINDOW_SIZE];
float ventralY_history[COV_WINDOW_SIZE];
float past_ventralY_history[COV_WINDOW_SIZE];

int16_t flowX;
int16_t flowY;

// How fast the gain ramps up
#define GAINRAMPX 2
#define GAINRAMPY 0.75

int16_t ventralX;
int16_t ventralY;

float cov_divX;
float cov_divY;

float errX;
float errY;

float pusedX;
float pusedY;

bool oscillatingX;
bool oscillatingY;
bool algoXY;

int32_t phi_des;
int32_t theta_des;

#define MAXBANK 25
//// Rest  ///////////////////////////////////////////

struct OpticalFlowTitus of_titusmodule;
float height;
int16_t thrustNoise;







// sending the divergence message to the ground station:
static void send_titusmodule(struct transport_tx *trans, struct link_device *dev)
{
	pprz_msg_send_TITUSMODULE(trans, dev, AC_ID,&divergence,&regular_divergence,&size_divergence,&ventralX,&ventralY,&flowX,&flowY,&cov_divX,&cov_divY,&cov_divZ,&height,&errX,&errY,&errZ,&pusedX,&pusedY,&pusedZ,&of_titusmodule.sum_errX,&of_titusmodule.sum_errY,&of_titusmodule.sum_errZ,&thrust,&phi_des,&theta_des,&test_sp_eu.phi,&test_sp_eu.theta,&test_sp_eu.psi,&nominalthrust,&heightEstimate);
}

/**
 * Calculate the difference from start till finish
 * @param[in] *starttime The start time to calculate the difference from
 * @param[in] *finishtime The finish time to calculate the difference from
 * @return The difference in milliseconds
 */
static uint32_t timeval_diff(struct timeval *starttime, struct timeval *finishtime)
{
	uint32_t msec;
	msec = (finishtime->tv_sec - starttime->tv_sec) * 1000;
	msec += (finishtime->tv_usec - starttime->tv_usec) / 1000;
	return msec;
}

//////////////////////////////////////////// Logging Module ///////////////////////////////////
void titusmodule_init(void)
{
	// Subscribe to ABI message
	AbiBindMsgOPTICAL_FLOW(TITUSMODULE_OPTICAL_FLOW_ID, &optical_flow_ev, titus_ctrl_optical_flow_cb);

	of_titusmodule.lp_factor = 0.6f;
	nominalthrust = GUIDANCE_V_NOMINAL_HOVER_THROTTLE;
	of_titusmodule.delay_steps = 40;
	vision_message_nr = 1;
	previous_message_nrXY = 0;
	previous_message_nrZ  = 0;
	ofmethode = OPTICFLOW_METHOD;
	height = 0;
	thrustNoise = 0;

	register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_TITUSMODULE, send_titusmodule);
}

void titusmodule_start(void)
{

}

void titusmodule_periodic(void)
{
	if((autopilot_get_mode()==autopilot_get_mode()) && (stabilization_cmd[COMMAND_THRUST]))
	{
		OptiThrust = stabilization_cmd[COMMAND_THRUST];
	}
}

void titusmodule_stop(void)
{

}
//////////////////////////////////////////// Control Module ///////////////////////////////////


// Init V & H
void h_ctrl_module_init(void)
{
	struct timespec spec;
	clock_gettime(CLOCK_REALTIME, &spec);
	previous_timeXY = spec.tv_nsec / 1.0E6;
	//previous_timeXY = time(NULL);

	dtXY = 0.0f;

	ind_histXY = 0;

	cov_divX = 0.0f;
	cov_divY = 0.0f;

	flowX = 0;
	flowY = 0;

	for (int i = 0; i < COV_WINDOW_SIZE; i++) {
		ventralX_history[i] = 0;
		ventralY_history[i] = 0;
		past_ventralX_history[i]=0;
		past_ventralY_history[i]=0;
	}

	magicfactorX = 0.008;
	magicfactorY = 0.008;
	oscphi = 1;
	osctheta = 0;

	startPusedX = 0.25;
	startPusedY = 0.25;

	rampX = 0.75;
	rampY = 0.75;

	covXthreshold = 30000;
	covYthreshold = 30000;

	performXY = 1;
	algoXY = performXY;

}

void v_ctrl_module_init(void)
{
	of_titusmodule.divergence_setpoint = 0.0f;
	of_titusmodule.sum_errZ = 0.0f;

	struct timespec spec;
	clock_gettime(CLOCK_REALTIME, &spec);
	previous_timeZ = spec.tv_nsec / 1.0E6;
	//previous_timeZ = time(NULL);

	// clear histories:
	ind_histZ = 0;
	for (int i = 0; i < COV_WINDOW_SIZE; i++) {
		//		thrust_history[i] = 0;
		divergence_history[i] = 0;
		past_divergence_history[i]=0;
	}

	divergence = 0;
	divergence_vision = 0.0f;
	cov_divZ = 0.0f;
	dtZ = 0.0f;

	setFFtime = 0;

	FFtime = 0.6;
	FFfactor = 1.1;

	rampZ = 1;

	startPusedZ = 0.4;

	testigainZ = 0.01;

	covZthreshold = 0.03;

	performFFtakeoff = 0;
	inFFtakeoff = performFFtakeoff;

	performZ = 1;
	algoZ = performZ;

	heightEstimate = 0;
}

// Read H RC
void guidance_h_module_read_rc(void)
{

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
		if(inFFtakeoff)
		{
			test_sp_eu.phi = 0;
			test_sp_eu.theta = 0;
			test_sp_eu.psi = 0;

			// Run the stabilization mode
			stabilization_attitude_set_rpy_setpoint_i(&test_sp_eu);
		}
		else
		{
			if(algoXY)
			{
				// Manage time stuff ////////////////////////////////////////////////////////////

				// ensure dt >= 0
				if (dtXY < 0) { dtXY = 0.0f; }

				// get delta time, dt, to scale the divergence measurements correctly when using "simulated" vision:
				struct timespec spec;
				clock_gettime(CLOCK_REALTIME, &spec);
				long new_time = spec.tv_nsec / 1.0E6;
				long delta_t = new_time - previous_timeXY;
				dtXY2 = ((float)delta_t) / 1000.0f;
				dtXY += dtXY2;
				if (dtXY > 10.0f) {
					dtXY = 0.0f;
					return;
				}
				previous_timeXY = new_time;


				// Update and filter vision ////////////////////////////////////////////////////////////

				float ventral_factor; // factor that maps divergence in pixels as received from vision to /frame
				if (vision_message_nr != previous_message_nrXY && dtXY > 1E-5 && ind_histXY > 1) {
					ventral_factor = -1.28f; // magic number comprising field of view etc.

					// TODO: Have a look if it should be floats or int16_t's
					// TODO: divide or not by dtXY?
					float new_ventralflowX = (flowX * ventral_factor) / dtXY;
					float new_ventralflowY = (flowY * ventral_factor) / dtXY;

					// TODO: Prevent large changes with the magic factors 200 and 100
					//						if (abs(new_ventralflowX - ventralX) > 200) {
					//							if (new_ventralflowX < ventralX) { new_ventralflowX = ventralX - 100; }
					//							else { new_ventralflowX = ventralX + 100; }
					//						}

					// low-pass filter the ventral flows:
					ventralX = ventralX * of_titusmodule.lp_factor + (new_ventralflowX * (1.0f - of_titusmodule.lp_factor));
					ventralY = ventralY * of_titusmodule.lp_factor + (new_ventralflowY * (1.0f - of_titusmodule.lp_factor));
					previous_message_nrXY = vision_message_nr;
					dtXY = 0.0f;
				} else {
					// after re-entering the module, the divergence should be equal to the set point:
					if (ind_histXY <= 1) {
						ind_histXY++;
						dtXY = 0.0f;
					}
					// else: do nothing, let dt increment
					return;
				}
				errX = of_titusmodule.ventralflow_setpoint - ventralX;
				errY = of_titusmodule.ventralflow_setpoint - ventralY;
				of_titusmodule.sum_errX += errX;
				of_titusmodule.sum_errY += errY;

				// Increase gain & set pitch and roll angles ///////////////////////////////////////////////////////////////////////

				if(!oscillatingX)
				{
					pusedX += dtXY2*rampX;
				}
				if(!oscillatingY)
				{
					pusedY += dtXY2*rampY;
				}
				// set desired pitch en roll
				if(oscphi)
				{
					phi_des = Max(-MAXBANK,Min(magicfactorX * pusedX * errX,MAXBANK)); //+ of_titusmodule.igain * of_titusmodule.sum_err;
				}
				else
				{
					phi_des = 0;
				}
				if(osctheta)
				{
					theta_des = Max(-MAXBANK,Min(magicfactorY * pusedY * errY,MAXBANK)); //+ of_titusmodule.igain * of_titusmodule.sum_err;
				}
				else
				{
					theta_des = 0;
				}

				test_sp_eu.phi = BFP_OF_REAL(RadOfDeg(phi_des), INT32_ANGLE_FRAC);
				test_sp_eu.theta = BFP_OF_REAL(RadOfDeg(theta_des), INT32_ANGLE_FRAC);

				// Log everything ////////////////////////////////////////////////////////////////////////

				ventralX_history[ind_histXY % COV_WINDOW_SIZE] = ventralX;
				ventralY_history[ind_histXY % COV_WINDOW_SIZE] = ventralY;


				int ind_past = (ind_histXY % COV_WINDOW_SIZE) - of_titusmodule.delay_steps;
				while (ind_past < 0) { ind_past += COV_WINDOW_SIZE; }

				past_ventralX_history[ind_histXY % COV_WINDOW_SIZE] = ventralX_history[ind_past];
				past_ventralY_history[ind_histXY % COV_WINDOW_SIZE] = ventralY_history[ind_past];
				ind_histXY++;

				// only take covariance into account if there are enough samples in the histories:
				if (ind_histXY >= COV_WINDOW_SIZE) {
					cov_divX = get_cov(past_ventralX_history, ventralX_history, COV_WINDOW_SIZE);
					cov_divY = get_cov(past_ventralY_history, ventralY_history, COV_WINDOW_SIZE);
				} else {
					cov_divX = 0;
					cov_divY = 0;
				}

				// Check for oscillations ////////////////////////////////////////////////////////////////////

				if(abs(cov_divX)>covXthreshold)
				{
					if(!oscillatingX)
					{
						oscillatingX = 1;
						pusedX = pusedX*0.75;
					}
				}
				if(cov_divY>covYthreshold)
				{
					if(!oscillatingY)
					{
						oscillatingY = 1;
						pusedY = pusedY*0.75;
					}
				}
			}
			else
			{
				test_sp_eu.phi = BFP_OF_REAL(RadOfDeg(0), INT32_ANGLE_FRAC);
				test_sp_eu.theta = BFP_OF_REAL(RadOfDeg(0), INT32_ANGLE_FRAC);

			}
			// Compute 1 or 2 horizontal axes with optitrack
			computeOptiTrack(!oscphi,!osctheta,&test_sp_eu);

			// Due to toiletboiling, keep heading 0 for now
			test_sp_eu.psi = 0;

			// Run the stabilization mode
			stabilization_attitude_set_rpy_setpoint_i(&test_sp_eu);
		}
	}
}



// Run H
void v_ctrl_module_run(bool in_flight)
{
	if (!in_flight)
	{
		//		printf("randW,%d,rand,%f\n",(int) round( ( (double) rand() ) / ( (double) RAND_MAX)*(100) ),round(rand()));
		// Reset integrators
		stabilization_cmd[COMMAND_THRUST] = 0;
	}
	else
	{
		if(algoZ)
		{
			if(inFFtakeoff)
			{
				//			struct timespec FFspec;
				//			clock_gettime(CLOCK_REALTIME, &FFspec);
				struct timeval FFspec;
				gettimeofday(&FFspec, NULL);


				if(!setFFtime)
				{
					setFFtime = 1;
					FF_starttime = FFspec;
				}
				else
				{
					uint32_t FF_time = timeval_diff(&FF_starttime,&FFspec);
					if(FF_time>(uint32_t)(FFtime*1500))
					{
						inFFtakeoff = 0;
					}
					else if(FF_time>(uint32_t)(FFtime*1000))
					{
						stabilization_cmd[COMMAND_THRUST] = nominalthrust * 0.95 * MAX_PPRZ;
					}
					else
					{
						stabilization_cmd[COMMAND_THRUST] = Max(nominalthrust * FFfactor,1) * MAX_PPRZ;
					}
				}
			}
			else
			{
				//				if (vision_message_nr != previous_message_nrZ)
				//				{
				if(ofmethode)
				{
					divergence_vision = regular_divergence;
				}
				else
				{
					divergence_vision = size_divergence;
				}

				int32_t nominal_throttle = nominalthrust * MAX_PPRZ;

				// Manage time stuff ////////////////////////////////////////////////////////////

				// ensure dt >= 0
				if (dtZ < 0) { dtZ = 0.0f; }

				// get delta time, dt
				struct timespec spec;
				clock_gettime(CLOCK_REALTIME, &spec);
				long new_time = spec.tv_nsec / 1.0E6;
				long delta_t = new_time - previous_timeZ;
				dtZ2 = ((float)delta_t) / 1000.0f;
				dtZ += dtZ2;
				if (dtZ > 10.0f) {
					dtZ = 0.0f;
					return;
				}
				previous_timeZ = new_time;


				// Update and filter vision ////////////////////////////////////////////////////////////

				float div_factor; // factor that maps divergence in pixels as received from vision to /frame
				//					if (dtZ > 1E-5 && ind_histZ > 1) {
				if (vision_message_nr != previous_message_nrZ && dtZ > 1E-5 && ind_histZ >= 1) {
					// TODO: this div_factor depends on the subpixel-factor (automatically adapt?)
					div_factor = -1.28f; // magic number comprising field of view etc.
					float new_divergence = (divergence_vision * div_factor) / dtZ;

					if (fabs(new_divergence - divergence) > 0.20) {
						if (new_divergence < divergence) { new_divergence = divergence - 0.10f; }
						else { new_divergence = divergence + 0.10f; }
					}
					// low-pass filter the divergence:
					divergence = divergence * of_titusmodule.lp_factor + (new_divergence * (1.0f - of_titusmodule.lp_factor));
					previous_message_nrZ = vision_message_nr;
					dtZ = 0.0f;
				} else {
					// after re-entering the module, the divergence should be equal to the set point:
					if (ind_histZ < 1) {
						ind_histZ++;
						dtZ = 0.0f;
					}
					// else: do nothing, let dt increment
					return;
				}
				errZ = of_titusmodule.divergence_setpoint - divergence;
				of_titusmodule.sum_errZ += errZ;

				// Increase gain ///////////////////////////////////////////////////////////////////////

//				if((!oscillatingZ) && (ind_histZ >= 1*COV_WINDOW_SIZE+of_titusmodule.delay_steps))
					if((!oscillatingZ) && (ind_histZ >= 1*COV_WINDOW_SIZE))
					{
						pusedZ += dtZ2*rampZ;
					}

				// TODO: nominal throttle compensated for body angles cos?
				thrust = (nominal_throttle) + pusedZ * errZ * MAX_PPRZ + testigainZ * of_titusmodule.sum_errZ * MAX_PPRZ;
				//				+0.01*MAX_PPRZ

				// Log everything ////////////////////////////////////////////////////////////////////////

				//					normalized_thrust = (float)(thrust / (MAX_PPRZ / 100));
				//					thrust_history[ind_histZ % COV_WINDOW_SIZE] = normalized_thrust;
				divergence_history[ind_histZ % COV_WINDOW_SIZE] = divergence;
				int ind_past = (ind_histZ % COV_WINDOW_SIZE) - of_titusmodule.delay_steps;
				while (ind_past < 0) { ind_past += COV_WINDOW_SIZE; }
				float past_divergence = divergence_history[ind_past];
				past_divergence_history[ind_histZ % COV_WINDOW_SIZE] = 100.0f * past_divergence;
				ind_histZ++;

				// only take covariance into account if there are enough samples in the histories:
//				if (ind_histZ >= 1*COV_WINDOW_SIZE+of_titusmodule.delay_steps) {
					if (ind_histZ >= 1*COV_WINDOW_SIZE) {

						//						cov_divZ = get_cov(thrust_history, divergence_history, COV_WINDOW_SIZE);
						cov_divZ = get_cov(past_divergence_history, divergence_history, COV_WINDOW_SIZE);
					}
					else {
						cov_divZ = 0;
					}

					// Check for oscillations ////////////////////////////////////////////////////////////////////
					//	TODO: Remove FABS?
					if(fabs(cov_divZ)>covZthreshold)
					{
						if(!oscillatingZ)
						{
							oscillatingZ = 1;
							algoXY = 1;
							//						heightEstimate = (20/3 * pusedZ) - 19/6;
							printf("Height,%f,Gain,%f\n",height,pusedZ);
							pusedZ = pusedZ*0.75;


							// Test setting XY
							//						pusedX = 0.75*(0.1*heightEstimate +0.35);
							//						pusedY = 0.75*(0.1*heightEstimate +0.35);
							//						oscillatingX = 1;
							//						oscillatingY = 1;


						}
					}

					// Add Noise? ///////////////////////////////////////////
					thrustNoise =  (int) round( ( (double) rand() ) / ( (double) RAND_MAX)*(200) ) -100;




					// Set thrust ///////////////////////////////////////////////////////////////////////////
					//					previous_message_nrZ = vision_message_nr;
					//				}
					stabilization_cmd[COMMAND_THRUST] = thrust + thrustNoise;

				}
			}
			else
			{
				thrust = nominalthrust * MAX_PPRZ;
			}
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

		// Set current psi as heading
		test_sp_eu.psi = stateGetNedToBodyEulers_i()->psi;

		VECT2_COPY(titusmodule_ref_pos, *stateGetPositionNed_i());
		//	stabilization_attitude_set_failsafe_setpoint();

		// reset integrator
		of_titusmodule.sum_errX = 0.0f;

		oscillatingX = 0;
		oscillatingY = 0;
		ind_histXY = 0;
		pusedX = startPusedX;
		pusedY = startPusedY;
		cov_divX = 0;
		cov_divY = 0;

		ventralX = of_titusmodule.ventralflow_setpoint;
		ventralY = of_titusmodule.ventralflow_setpoint;

		dtXY = 0.0f;
		dtXY2 = 0.0f;

		flowX = 0;
		flowY = 0;

		struct timespec spec;
		clock_gettime(CLOCK_REALTIME, &spec);
		previous_timeXY = spec.tv_nsec / 1.0E6;
		vision_message_nr = 1;
		previous_message_nrXY = 0;
		previous_message_nrZ  = 0;

		for (int i = 0; i < COV_WINDOW_SIZE; i++) {
			ventralX_history[i] = 0;
			ventralY_history[i] = 0;
			past_ventralX_history[i]=0;
			past_ventralY_history[i]=0;
		}

		inFFtakeoff = performFFtakeoff;

		algoXY = performXY;
	}

	void guidance_h_module_run(bool in_flight)
	{
		if(electrical.bat_low)
		{
			autopilot_static_set_mode(AP_MODE_NAV);
		}
		else
		{
			h_ctrl_module_run(in_flight);

			stabilization_attitude_run(in_flight);
		}
	}



	// Implement own Vertical loops
	void guidance_v_module_init(void)
	{
		// initialization of your custom vertical controller goes here
		v_ctrl_module_init();
	}

	void guidance_v_module_enter(void)
	{
		// reset integrator
		of_titusmodule.sum_errZ = 0.0f;

		oscillatingZ = 0;
		ind_histZ = 0;
		pusedZ = startPusedZ;
		cov_divZ = 0;
		divergence = of_titusmodule.divergence_setpoint;
		dtZ = 0.0f;
		dtZ2 = 0.0f;
		struct timespec spec;
		clock_gettime(CLOCK_REALTIME, &spec);
		previous_timeZ = spec.tv_nsec / 1.0E6;
		vision_message_nr = 1;
		previous_message_nrXY = 0;
		previous_message_nrZ  = 0;
		for (int i = 0; i < COV_WINDOW_SIZE; i++) {
			//		thrust_history[i] = 0;
			divergence_history[i] = 0;
			past_divergence_history[i]=0;
		}

		nominalthrust = OptiThrust/MAX_PPRZ;
		stabilization_cmd[COMMAND_THRUST] = nominalthrust * MAX_PPRZ;

		setFFtime = 0;
		inFFtakeoff = performFFtakeoff;
		algoZ = performZ;
	}

	void guidance_v_module_run(bool in_flight)
	{
		if(electrical.bat_low)
		{
			autopilot_static_set_mode(AP_MODE_NAV);
		}
		else
		{
			height = (stateGetPositionEnu_i()->z)*0.0039063;
			// your vertical controller goes here
			v_ctrl_module_run(in_flight);
		}
	}


	/**
	 * Get the desired Euler angles for optitrack stabilization
	 * @param[in] Boolean whether to Phi or not
	 * @param[in] Boolean whether to Theta or not
	 * @param[out] The desired Euler angles
	 */
	void computeOptiTrack(bool phi,bool theta,struct Int32Eulers *opti_sp_eu)
	{

		bool optiVelOnly;
		optiVelOnly = 0;

		// Heading is going wrong?
		int32_t psi = stateGetNedToBodyEulers_i()->psi;

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
		PPRZ_ITRIG_SIN(s_psi, psi);
		PPRZ_ITRIG_COS(c_psi, psi);

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



	static void titus_ctrl_optical_flow_cb(uint8_t sender_id, uint32_t stamp, int16_t flow_x, int16_t flow_y, int16_t flow_der_x, int16_t flow_der_y, float quality, float size_div,float regular_div, float dist)
	{
		//	height = dist;

		flowX = flow_x;
		flowY = flow_y;

		regular_divergence = regular_div;
		size_divergence = size_div;

		vision_message_nr++;
		if (vision_message_nr > 10) { vision_message_nr = 0; }
	}
