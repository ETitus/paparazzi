#include "optical_flow_hover.h"

//#include "generated/airframe.h"
#include "paparazzi.h"
#include "subsystems/abi.h"
#include "firmwares/rotorcraft/stabilization.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude.h"
//#include "firmwares/rotorcraft/guidance/guidance_v_adapt.h"
#include "subsystems/electrical.h"
#include <stdio.h>


#include "subsystems/datalink/telemetry.h"
//
//// for measuring time
#include "mcu_periph/sys_time.h"
//
// Additional math functions
#include "math/pprz_stat.h"

/* Use optical flow estimates */
#ifndef OFH_OPTICAL_FLOW_ID
#define OFH_OPTICAL_FLOW_ID ABI_BROADCAST
#endif
PRINT_CONFIG_VAR(OFH_OPTICAL_FLOW_ID)

#ifndef OFH_LP_CONST
#define OFH_LP_CONST 0.4 // EF
//#define OFH_LP_CONST 0.6 // LK
#endif

#ifndef OFL_COV_METHOD
#define OFL_COV_METHOD 0
#endif

// number of time steps used for calculating the covariance (oscillations) and the delay steps
#ifndef OFH_COV_WINDOW_SIZE
#define OFH_COV_WINDOW_SIZE (10*10)
#endif

#ifndef OFH_COV_DELAY_STEPS
#define OFH_COV_DELAY_STEPS (5*10)
#endif

#ifndef OFH_PGAINZ
#define OFH_PGAINZ 0.4 // EF
//#define OFH_PGAINZ 0.40 // LK
#endif

#ifndef OFH_IGAINZ
#define OFH_IGAINZ 0.005 // EF
//#define OFH_IGAINZ 0.005 // LK
#endif

#ifndef OFH_DGAINZ
#define OFH_DGAINZ 0.0
#endif

#ifndef OFH_RAMPZ
#define OFH_RAMPZ 0.05 // EF
//#define OFH_RAMPZ 0.05 // LK
#endif

#ifndef OFH_REDUCTIONZ
#define OFH_REDUCTIONZ 0.5
#endif

#ifndef OFH_COVDIV_SETPOINT
#define OFH_COVDIV_SETPOINT -0.06 // EF
//#define OFH_COVDIV_SETPOINT -0.02 // LK
#endif



#ifndef OFH_PGAINX
#define OFH_PGAINX 0.0
#endif

#ifndef OFH_IGAINX
#define OFH_IGAINX 0.0001
//#define OFH_IGAINX 0.0
#endif

#ifndef OFH_DGAINX
#define OFH_DGAINX 0.0
#endif

#ifndef OFH_PGAINY
#define OFH_PGAINY 0.0
#endif

#ifndef OFH_IGAINY
#define OFH_IGAINY 0.0001
#endif

#ifndef OFH_DGAINY
#define OFH_DGAINY 0.0
#endif

#ifndef OFH_RAMPXY
#define OFH_RAMPXY 0.0004
#endif

#ifndef OFH_REDUCTIONXY
#define OFH_REDUCTIONXY 0.4
#endif

#ifndef OFH_COVFLOW_SETPOINT
#define OFH_COVFLOW_SETPOINT -2000
#endif
// variables retained between module calls
float vision_time, prev_vision_timeXY, prev_vision_timeZ;

bool oscillatingX;
bool oscillatingY;
int16_t flowX;
int16_t flowY;
uint32_t ind_histXY;
//uint8_t cov_array_filledXY;
float cov_flowX = 0;
float cov_flowY = 0;
float flowX_history[OFH_COV_WINDOW_SIZE];
float flowY_history[OFH_COV_WINDOW_SIZE];
float past_flowX_history[OFH_COV_WINDOW_SIZE];
float past_flowY_history[OFH_COV_WINDOW_SIZE];
float phi_history[OFH_COV_WINDOW_SIZE];
float theta_history[OFH_COV_WINDOW_SIZE];

float pusedX;
float pusedY;

// Stabilizing commands
struct Int32Eulers ofh_sp_eu;

float phi_des;
float theta_des;

#define MAXBANK 10.0

float phi_des_start;
float theta_des_start;

bool oscillatingZ;
float divergence_vision;
float regular_divergence;
float size_divergence;
float thrust_history[OFH_COV_WINDOW_SIZE];
float divergence_history[OFH_COV_WINDOW_SIZE];
float past_divergence_history[OFH_COV_WINDOW_SIZE];
uint32_t ind_histZ;
uint8_t cov_array_filledZ;
float normalized_thrust;
float cov_divZ;
int32_t thrust_set;

float pusedZ;
float height;

// The optical flow ABI event
static abi_event optical_flow_ev;

// struct containing most relevant parameters
struct OpticalFlowHover of_hover_ctrl;

// sending the divergence message to the ground station:


/// Function definitions
// Callback function of the optical flow estimate:
void ofh_optical_flow_cb(uint8_t sender_id __attribute__((unused)), uint32_t stamp, int16_t flow_x, int16_t flow_y, int16_t flow_der_x, int16_t flow_der_y, float quality, float size_div,float regular_div, float dist);

// common functions for different hover strategies:
static void set_cov_div(int32_t thrust);
static void set_cov_flow(void);
static int32_t PID_divergence_control(float divergence_setpoint, float P, float I, float D, float dt);
float PID_flow_control(float setpoint, float P, float I, float D, float dt, bool phitheta);
static void update_errors(float error, float dt, uint8_t mode);

// resetting all variables to be called for instance when starting up / re-entering module
static void reset_horizontal_vars(void);
static void reset_vertical_vars(void);
void vertical_ctrl_module_init(void);
void vertical_ctrl_module_run(bool in_flight);
void horizontal_ctrl_module_init(void);
void horizontal_ctrl_module_run(bool in_flight);
void guidance_h_module_read_rc(void);


// Compute OptiTrack stabilization for 1/2 axes
void computeOptiTrack(bool phi, bool theta,struct Int32Eulers *opti_sp_eu);
#ifndef GH_GAIN_SCALE
#define GH_GAIN_SCALE 2
#endif
#ifndef MAX_POS_ERR
#define MAX_POS_ERR POS_BFP_OF_REAL(16.)
#endif
#ifndef MAX_SPEED_ERR
#define MAX_SPEED_ERR SPEED_BFP_OF_REAL(16.)
#endif
struct Int32Vect2 of_hover_pos_err;
struct Int32Vect2 of_hover_speed_err;
struct Int32Vect2 of_hover_ref_pos;
struct Int32Vect2 of_hover_trim_att_integrator;
struct Int32Vect2 of_hover_cmd_earth;





// Temporary stuff depending whether it works or not
uint32_t elc_time_start;
int32_t count_covdiv;
float lp_cov_div;
bool isplus;





// sending the divergence message to the ground station:
static void send_optical_flow_hover(struct transport_tx *trans, struct link_device *dev)
{
	height = (stateGetPositionEnu_i()->z)*0.0039063;
	pprz_msg_send_OPTICAL_FLOW_HOVER(trans, dev, AC_ID,&(of_hover_ctrl.flowX),&(of_hover_ctrl.flowY),&(of_hover_ctrl.divergence),
			&cov_flowX,&cov_flowY,&cov_divZ,&pusedX,&pusedY,&pusedZ,&(of_hover_ctrl.sum_errX),&(of_hover_ctrl.sum_errY),&(of_hover_ctrl.sum_errZ),
			&thrust_set,&phi_des,&theta_des,&height);
}


// Init the optical flow hover module
void optical_flow_hover_init()
{
	of_hover_ctrl.lp_const = OFH_LP_CONST;
	Bound(of_hover_ctrl.lp_const, 0.001f, 1.f);
	of_hover_ctrl.delay_steps = OFH_COV_DELAY_STEPS;
	of_hover_ctrl.window_size = OFH_COV_WINDOW_SIZE;
	of_hover_ctrl.ofmethode = OPTICFLOW_METHOD;

	of_hover_ctrl.divergence_setpoint = 0.0f;
	of_hover_ctrl.flow_setpoint = 0.0f;

	of_hover_ctrl.pgainZ = OFH_PGAINZ;
	of_hover_ctrl.igainZ = OFH_IGAINZ;
	of_hover_ctrl.dgainZ = OFH_DGAINZ;
	of_hover_ctrl.rampZ  = OFH_RAMPZ;

	of_hover_ctrl.pgainX = OFH_PGAINX;
	of_hover_ctrl.igainX = OFH_IGAINX;
	of_hover_ctrl.dgainX = OFH_DGAINX;

	of_hover_ctrl.pgainY = OFH_PGAINY;
	of_hover_ctrl.igainY = OFH_IGAINY;
	of_hover_ctrl.dgainY = OFH_DGAINY;

	of_hover_ctrl.rampXY  = OFH_RAMPXY;

	of_hover_ctrl.reduction_factorZ = OFH_REDUCTIONZ;
	of_hover_ctrl.reduction_factorXY = OFH_REDUCTIONXY;

	of_hover_ctrl.covFlow_set_point = OFH_COVFLOW_SETPOINT;
	of_hover_ctrl.covDiv_set_point  = OFH_COVDIV_SETPOINT;

	of_hover_ctrl.COV_METHOD = OFL_COV_METHOD;

	oscphi = 1;
	osctheta = 1;

	reset_horizontal_vars();
	reset_vertical_vars();

	// Subscribe to the optical flow estimator:
	AbiBindMsgOPTICAL_FLOW(OFH_OPTICAL_FLOW_ID, &optical_flow_ev, ofh_optical_flow_cb);

	// register telemetry:
	register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_OPTICAL_FLOW_HOVER, send_optical_flow_hover);
}

// Start the optical flow hover module
void optical_flow_hover_start(void){}
// Run the optical flow hover module
void optical_flow_hover_periodic(void){}
// Stop the optical flow hover module
void optical_flow_hover_stop(void){}

/**
 * Initialize the vertical optical flow hover module
 */
void vertical_ctrl_module_init(void)
{
	// filling the of_hover_ctrl struct with default values:
	reset_vertical_vars();
}

/**
 * Initialize the horizontal optical flow hover module
 */
void horizontal_ctrl_module_init(void)
{
	// filling the of_hover_ctrl struct with default values:
	reset_horizontal_vars();
}

/**
 * Reset all horizontal variables:
 */
static void reset_horizontal_vars(void)
{
	struct Int32Eulers tempangle;
	int32_eulers_of_quat(&tempangle,&stab_att_sp_quat);
	phi_des_start = DegOfRad(FLOAT_OF_BFP(tempangle.phi,INT32_ANGLE_FRAC));;
	theta_des_start = DegOfRad(FLOAT_OF_BFP(tempangle.theta,INT32_ANGLE_FRAC));;

	phi_des = 0;
	theta_des = 0;

	oscillatingX = 0;
	oscillatingY = 0;
	flowX = 0;
	flowY = 0;

	of_hover_ctrl.sum_errX = 0.0f;
	of_hover_ctrl.d_errX = 0.0f;
	of_hover_ctrl.previous_errX = 0.0f;

	of_hover_ctrl.sum_errY = 0.0f;
	of_hover_ctrl.d_errY = 0.0f;
	of_hover_ctrl.previous_errY = 0.0f;

	pusedX = of_hover_ctrl.pgainX;
	pusedY = of_hover_ctrl.pgainY;

	ofh_sp_eu.phi = phi_des_start;
	ofh_sp_eu.phi = theta_des_start;

	ind_histXY = 0;
	//	cov_array_filledXY = 0;

	cov_flowX = 0.0f;
	cov_flowY = 0.0f;

	for(uint16_t i=0;i<of_hover_ctrl.window_size;i++)
	{
		flowX_history[i] = 0.0f;
		flowY_history[i] = 0.0f;
		past_flowX_history[i] = 0.0f;
		past_flowY_history[i] = 0.0f;
		phi_history[i] = 0.0f;
		theta_history[i] = 0.0f;
	}

	of_hover_ctrl.flowX = 0;
	of_hover_ctrl.flowY = 0;

	vision_time = get_sys_time_float();
	prev_vision_timeXY = vision_time;
}

/**
 * Reset all vertical variables:
 */
static void reset_vertical_vars(void)
{
	oscillatingZ = 0;

	regular_divergence = 0;
	size_divergence = 0;

	for(uint16_t i=0;i<of_hover_ctrl.window_size;i++)
	{
		divergence_history[i] = 0.0f;
		thrust_history[i] = 0.0f;
		past_divergence_history[i] = 0.0f;
	}

	normalized_thrust = 0;

	ind_histZ = 0;

	pusedZ = of_hover_ctrl.pgainZ;


	// Temporary stuff depending on if it works
	lp_cov_div = 0.0f;
	count_covdiv = 0;

	cov_divZ = 0.0f;
	cov_array_filledZ = 0;

	of_hover_ctrl.sum_errZ = 0.0f;
	of_hover_ctrl.d_errZ = 0.0f;
	of_hover_ctrl.previous_errZ = 0.0f;

	vision_time = get_sys_time_float();
	prev_vision_timeZ = vision_time;

	elc_time_start = vision_time;
	isplus = 0;
	of_hover_ctrl.divergence = 0;

	height = (stateGetPositionEnu_i()->z)*0.0039063;
}

// Read H RC
void guidance_h_module_read_rc(void){}

/**
 * Run the horizontal optical flow hover module
 */
void horizontal_ctrl_module_run(bool in_flight)
{
	/***********
	 * TIME
	 ***********/
	float ventral_factor = -1.28f; // magic number comprising field of view etc.

	float dt = vision_time - prev_vision_timeXY;

	// check if new measurement received
	if (dt <= 1e-5f) {
		return;
	}

	/***********
	 * VISION
	 ***********/

	Bound(of_hover_ctrl.lp_const, 0.001f, 1.f);
	float lp_factor = dt / of_hover_ctrl.lp_const;
	Bound(lp_factor, 0.f, 1.f);

	float new_flowX = (flowX * ventral_factor) / dt;
	float new_flowY = (flowY * ventral_factor) / dt;

	//TODO: deal with (unlikely) fast changes in Flow?

	// low-pass filter the divergence:
	of_hover_ctrl.flowX += (new_flowX - of_hover_ctrl.flowX) * lp_factor;
	of_hover_ctrl.flowY += (new_flowY - of_hover_ctrl.flowY) * lp_factor;
	//	prev_vision_timeXY = vision_time;

	/***********
	 * CONTROL
	 ***********/
	if(!oscillatingX)
	{
		// if not oscillating, increase gain
		pusedX += of_hover_ctrl.rampXY*dt;
	}
	if(!oscillatingY)
	{
		// if not oscillating, increase gain
		pusedY += of_hover_ctrl.rampXY*dt;
	}

	// set desired pitch en roll
	if(oscphi)
	{
		phi_des = phi_des_start + PID_flow_control(of_hover_ctrl.flow_setpoint, pusedX, of_hover_ctrl.igainX, of_hover_ctrl.dgainX, dt, 0);
	}
	if(osctheta)
	{
		theta_des = theta_des_start + PID_flow_control(of_hover_ctrl.flow_setpoint, pusedY, of_hover_ctrl.igainY, of_hover_ctrl.dgainY, dt, 1);
	}

	// update covariance
	set_cov_flow();

	ofh_sp_eu.phi = BFP_OF_REAL(RadOfDeg(phi_des*oscphi), INT32_ANGLE_FRAC);
	ofh_sp_eu.theta = BFP_OF_REAL(RadOfDeg(theta_des*osctheta), INT32_ANGLE_FRAC);

	// Check for oscillations
	if( (cov_flowX<of_hover_ctrl.covFlow_set_point) && (!oscillatingX) )
	{
		oscillatingX = 1;
		pusedX = pusedX*of_hover_ctrl.reduction_factorXY;
	}
	if( (cov_flowY<of_hover_ctrl.covFlow_set_point) && (!oscillatingY) )
	{
		oscillatingY = 1;
		pusedY = pusedY*of_hover_ctrl.reduction_factorXY;
	}

	// Compute 0, 1 or 2 horizontal axes with optitrack
	computeOptiTrack(!oscphi,!osctheta,&ofh_sp_eu);

	// Run the stabilization mode
	stabilization_attitude_set_rpy_setpoint_i(&ofh_sp_eu);
	prev_vision_timeXY = vision_time;

}

/**
 * Run the vertical optical flow hover module
 */
void vertical_ctrl_module_run(bool in_flight)
{
	/***********
	 * TIME
	 ***********/

	float div_factor; // factor that maps divergence in pixels as received from vision to 1 / frame

	float dt = vision_time - prev_vision_timeZ;

	// check if new measurement received
	if (dt <= 1e-5f) {
		return;
	}

	/***********
	 * VISION
	 ***********/

	Bound(of_hover_ctrl.lp_const, 0.001f, 1.f);
	float lp_factor = dt / of_hover_ctrl.lp_const;
	Bound(lp_factor, 0.f, 1.f);

	// Vision
	div_factor = -1.28f; // magic number comprising field of view etc.
	float new_divergence = (divergence_vision * div_factor) / dt;

	// deal with (unlikely) fast changes in divergence:
	static const float max_div_dt = 0.20f;
	if (fabsf(new_divergence - of_hover_ctrl.divergence) > max_div_dt) {
		if (new_divergence < of_hover_ctrl.divergence) { new_divergence = of_hover_ctrl.divergence - max_div_dt; }
		else { new_divergence = of_hover_ctrl.divergence + max_div_dt; }
	}

	// low-pass filter the divergence:
	of_hover_ctrl.divergence += (new_divergence - of_hover_ctrl.divergence) * lp_factor;
	prev_vision_timeZ = vision_time;

	/***********
	 * CONTROL
	 ***********/

	if(1)
	{
		if(!oscillatingZ)
		{
			// if not oscillating, increase gain
			pusedZ += of_hover_ctrl.rampZ*dt;
		}


		if(0)
		{
			if((get_sys_time_float() - elc_time_start) >= 1.0f)
			{
				if(isplus)
				{
					of_hover_ctrl.divergence_setpoint = 0.01f;
					isplus = 0;
				}
				else
				{
					of_hover_ctrl.divergence_setpoint = -0.01f;
					isplus = 1;
				}
				elc_time_start = get_sys_time_float();
			}
		}


		// use the divergence for control:
		thrust_set = PID_divergence_control(of_hover_ctrl.divergence_setpoint, pusedZ, of_hover_ctrl.igainZ, of_hover_ctrl.dgainZ, dt);

		// Check for oscillations
		if(cov_divZ<of_hover_ctrl.covDiv_set_point && (!oscillatingZ))
		{
			oscillatingZ = 1;
			of_hover_ctrl.divergence_setpoint = 0.0f;
			printf("Height,%f,Gain,%f\n",height,pusedZ);
			pusedZ = pusedZ*of_hover_ctrl.reduction_factorZ;
		}
	}
	else
	{
		// if not yet oscillating, increase the gains:
		if (!oscillatingZ && cov_divZ > of_hover_ctrl.covDiv_set_point) {
			float increasedGainZ = of_hover_ctrl.rampZ*dt;
			float gain_factor = pusedZ / (pusedZ+increasedGainZ);
			pusedZ += increasedGainZ;
			of_hover_ctrl.igainZ *= gain_factor;
			of_hover_ctrl.dgainZ *= gain_factor;
		}

		// use the divergence for control:
		thrust_set = PID_divergence_control(of_hover_ctrl.divergence_setpoint, pusedZ, of_hover_ctrl.igainZ, of_hover_ctrl.dgainZ, dt);


		// low pass filter cov div and remove outliers:
		if (fabsf(lp_cov_div - cov_divZ) < 2.2) {
			lp_cov_div = 0.99f * lp_cov_div + (1 - 0.99f) * cov_divZ;
		}
		// if oscillating, maintain a counter to see if it endures:
		if (lp_cov_div <= -0.0075) {
			count_covdiv++;
		} else {
			count_covdiv = 0;
			elc_time_start = get_sys_time_float();
		}
		// if the drone has been oscillating long enough, start landing:
		if (!oscillatingZ && (count_covdiv > 0 && (get_sys_time_float() - elc_time_start) >= 1.5f)) {

			// Oscillating:
			oscillatingZ = 1;
			printf("Height,%f,Gain,%f\n",height,pusedZ);

			// we don't want to oscillate, so reduce the gain:
			pusedZ = of_hover_ctrl.reduction_factorZ * pusedZ;
			of_hover_ctrl.igainZ = of_hover_ctrl.reduction_factorZ * of_hover_ctrl.igainZ;
			of_hover_ctrl.dgainZ = of_hover_ctrl.reduction_factorZ * of_hover_ctrl.dgainZ;
		}
	}
	stabilization_cmd[COMMAND_THRUST] = thrust_set;
}



/**
 * Set the covariance of the flow and past flow, possible the desired angle and the flow
 * This funciton should only be called once per time step
 * @param[in] thrust: the current thrust value
 */
void set_cov_flow(void)
{
	// histories and cov detection:
	flowX_history[ind_histXY] = of_hover_ctrl.flowX;
	flowY_history[ind_histXY] = of_hover_ctrl.flowY;

	int ind_past = ind_histXY - of_hover_ctrl.delay_steps;
	while (ind_past < 0) { ind_past += of_hover_ctrl.window_size; }
	past_flowX_history[ind_histXY] = flowX_history[ind_past];
	past_flowY_history[ind_histXY] = flowY_history[ind_past];
	float normalized_phi = (float)(phi_des / (MAXBANK / 100.0));
	float normalized_theta = (float)(theta_des / (MAXBANK / 100.0));
	phi_history[ind_histXY] = normalized_phi;
	theta_history[ind_histXY] = normalized_theta;
	//	printf("ind: %d, pastInd: %d, VT: %f, dt: %f\n",ind_histXY,ind_past,vision_time,(vision_time - prev_vision_timeXY));

	// determine the covariance for hover detection:
	// only take covariance into account if there are enough samples in the histories:
	//	if (of_hover_ctrl.COV_METHOD == 0 && cov_array_filledXY > 0) {
	//		// TODO: step in hover set point causes an incorrectly perceived covariance
	//		cov_divZ = covariance_f(thrust_history, divergence_history, of_hover_ctrl.window_size);
	//	} else if (of_hover_ctrl.COV_METHOD == 1 && cov_array_filledXY > 1){
	//	if (cov_array_filledXY > 1){
	// todo: delay steps should be invariant to the run frequency
	cov_flowX = covariance_f(phi_history, flowX_history, of_hover_ctrl.window_size);
	// Temporarily set covFlowY to log both
	//	cov_flowY =covariance_f(past_flowX_history, flowX_history, of_hover_ctrl.window_size);
	//		cov_flowX = covariance_f(past_flowX_history, flowX_history, of_hover_ctrl.window_size);

	cov_flowY = covariance_f(theta_history, flowY_history, of_hover_ctrl.window_size);
	//		cov_flowY = covariance_f(past_flowY_history, flowY_history, of_hover_ctrl.window_size);
	//	}
	//	else
	//	{
	//		cov_flowX = 1000;
	//		cov_flowY = 1000;
	//	}

	//	if (cov_array_filledXY < 2 && ind_histXY + 1 == of_hover_ctrl.window_size) {
	//		cov_array_filledXY++;
	//	}
	ind_histXY = (ind_histXY + 1) % of_hover_ctrl.window_size;
}



/**
 * Set the covariance of the divergence and the thrust / past divergence
 * This funciton should only be called once per time step
 * @param[in] thrust: the current thrust value
 */
void set_cov_div(int32_t thrust)
{
	// histories and cov detection:
	divergence_history[ind_histZ] = of_hover_ctrl.divergence;

	normalized_thrust = (float)(thrust / (MAX_PPRZ / 100));
	thrust_history[ind_histZ] = normalized_thrust;

	int ind_past = ind_histZ - of_hover_ctrl.delay_steps;
	while (ind_past < 0) { ind_past += of_hover_ctrl.window_size; }
	past_divergence_history[ind_histZ] = divergence_history[ind_past];

	// determine the covariance for hover detection:
	// only take covariance into account if there are enough samples in the histories:
	if (of_hover_ctrl.COV_METHOD == 0 && cov_array_filledZ > 0) {
		// TODO: step in hover set point causes an incorrectly perceived covariance
		cov_divZ = covariance_f(thrust_history, divergence_history, of_hover_ctrl.window_size);
		// temporarily set cov_flowX here to log both cov divs at the same time
		//		cov_flowX = covariance_f(past_divergence_history, divergence_history, of_hover_ctrl.window_size);
	} else if (of_hover_ctrl.COV_METHOD == 1 && cov_array_filledZ > 1) {
		// todo: delay steps should be invariant to the run frequency
		cov_divZ = covariance_f(past_divergence_history, divergence_history, of_hover_ctrl.window_size);
	}

	if (cov_array_filledZ < 2 && ind_histZ + 1 == of_hover_ctrl.window_size) {
		cov_array_filledZ++;
	}
	ind_histZ = (ind_histZ + 1) % of_hover_ctrl.window_size;
}

/**
 * Determine and set the desired angle for constant flow control
 * @param[out] desired angle
 * @param[in] flow_set_point: The desired flow
 * @param[in] P: P-gain
 * @param[in] I: I-gain
 * @param[in] D: D-gain
 * @param[in] dt: time difference since last update
 */
float PID_flow_control(float setpoint, float P, float I, float D, float dt, bool phitheta)
{
	float des_angle = 0;

	if(!phitheta)
	{
		// determine the error:
		float err = setpoint - of_hover_ctrl.flowX;

		// update the controller errors:
		update_errors(err, dt, phitheta);

		// compute the desired angle
		des_angle = Max(-MAXBANK,Min(P * err + I * of_hover_ctrl.sum_errX + D * of_hover_ctrl.d_errX,MAXBANK));
	}
	else
	{
		// determine the error:
		float err = setpoint - of_hover_ctrl.flowY;

		// update the controller errors:
		update_errors(err, dt, phitheta);

		// compute the desired angle
		des_angle = Max(-MAXBANK,Min(P * err + I * of_hover_ctrl.sum_errY + D * of_hover_ctrl.d_errY,MAXBANK));
	}

	return des_angle;
}

/**
 * Determine and set the thrust for constant divergence control
 * @param[out] thrust
 * @param[in] divergence_set_point: The desired divergence
 * @param[in] P: P-gain
 * @param[in] I: I-gain
 * @param[in] D: D-gain
 * @param[in] dt: time difference since last update
 */
int32_t PID_divergence_control(float setpoint, float P, float I, float D, float dt)
{
	// determine the error:
	float err = setpoint - of_hover_ctrl.divergence;

	// update the controller errors:
	update_errors(err, dt, 2);


	//	struct Int32Eulers tempangle;
	//	  int32_eulers_of_quat(&tempangle,&stab_att_sp_quat);
	//	  float offangle = FLOAT_OF_BFP(tempangle.phi,INT32_ANGLE_FRAC);

	// PID control:
	int32_t thrust = (of_hover_ctrl.nominal_thrust // /cos(offangle)
			+ P * err
			+ I * of_hover_ctrl.sum_errZ
			+ D * of_hover_ctrl.d_errZ) * MAX_PPRZ;

	// bound thrust:
	Bound(thrust, 0.25 * of_hover_ctrl.nominal_thrust * MAX_PPRZ, MAX_PPRZ);

	// update covariance
	set_cov_div(thrust);

	return thrust;
}

/**
 * Updates the integral and differential errors for PID control and sets the previous error
 * @param[in] err: the error of the divergence and divergence setpoint
 * @param[in] dt:  time difference since last update
 */
void update_errors(float err, float dt, uint8_t mode)
{
	float lp_factor = dt / of_hover_ctrl.lp_const;
	Bound(lp_factor, 0.f, 1.f);

	if(mode==0)
	{
		// mode = 0 = horizontal X

		// maintain the controller errors:
		of_hover_ctrl.sum_errX += err;
		of_hover_ctrl.d_errX += (((err - of_hover_ctrl.previous_errX) / dt) - of_hover_ctrl.d_errX) * lp_factor;
		of_hover_ctrl.previous_errX = err;
	}
	else if(mode==1)
	{
		// mode = 1 = Horizontal Y

		// maintain the controller errors:
		of_hover_ctrl.sum_errY += err;
		of_hover_ctrl.d_errY += (((err - of_hover_ctrl.previous_errY) / dt) - of_hover_ctrl.d_errY) * lp_factor;
		of_hover_ctrl.previous_errY = err;
	}
	else
	{
		// mode = 2 = Vertical

		// maintain the controller errors:
		of_hover_ctrl.sum_errZ += err;
		of_hover_ctrl.d_errZ += (((err - of_hover_ctrl.previous_errZ) / dt) - of_hover_ctrl.d_errZ) * lp_factor;
		of_hover_ctrl.previous_errZ = err;
	}
}

void ofh_optical_flow_cb(uint8_t sender_id __attribute__((unused)), uint32_t stamp, int16_t flow_x, int16_t flow_y, int16_t flow_der_x, int16_t flow_der_y, float quality, float size_div,float regular_div, float dist)
{
	//	height = dist;

	if(!derotated)
	{
		flowX = flow_x;
		flowY = flow_y;
	} else
	{
		flowX = flow_der_x;
		flowY = flow_der_y;
	}
	regular_divergence = regular_div;
	size_divergence = size_div;

	if(of_hover_ctrl.ofmethode)
	{
		divergence_vision = regular_divergence;
	}
	else
	{
		divergence_vision = size_divergence;
	}

	vision_time = ((float)stamp) / 1e6;
}

////////////////////////////////////////////////////////////////////
// Call our vertical controller
void guidance_v_module_init(void)
{
	vertical_ctrl_module_init();
}

// Call our horizontal controller
void guidance_h_module_init(void)
{
	horizontal_ctrl_module_init();
}

/**
 * Entering the vertical module (user switched to module)
 */
void guidance_v_module_enter(void)
{
	reset_vertical_vars();

	// adaptive estimation - assume hover condition when entering the module
	of_hover_ctrl.nominal_thrust = (float) stabilization_cmd[COMMAND_THRUST] / MAX_PPRZ;
	thrust_set = of_hover_ctrl.nominal_thrust * MAX_PPRZ;
}

/**
 * Entering the horizontal module (user switched to module)
 */
void guidance_h_module_enter(void)
{
	// Set current psi as heading
	ofh_sp_eu.psi = stateGetNedToBodyEulers_i()->psi;

	VECT2_COPY(of_hover_ref_pos, *stateGetPositionNed_i());
	reset_horizontal_vars();

}

// Run the veritcal controller
void guidance_v_module_run(bool in_flight)
{
	if(electrical.bat_low)
	{
		autopilot_static_set_mode(AP_MODE_NAV);
	}
	else
	{
		// your vertical controller goes here
		vertical_ctrl_module_run(in_flight);
	}
}

// Run the horizontal controller
void guidance_h_module_run(bool in_flight)
{

	if(electrical.bat_low)
	{
		autopilot_static_set_mode(AP_MODE_NAV);
	}
	else
	{
		horizontal_ctrl_module_run(in_flight);
		stabilization_attitude_run(in_flight);
	}
}

/**
 * Get the desired Euler angles for optitrack stabilization
 * @param[in] Boolean whether to Phi or not
 * @param[in] Boolean whether to Theta or not
 * @param[out] The desired Euler angles
 */
void computeOptiTrack(bool phi, bool theta,struct Int32Eulers *opti_sp_eu)
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
	VECT2_DIFF(of_hover_pos_err, of_hover_ref_pos, pos_from_GPS);
	/* saturate it               */
	VECT2_STRIM(of_hover_pos_err, -MAX_POS_ERR, MAX_POS_ERR);

	struct Int32Vect2 ref_speed;
	ref_speed.x = 0;
	ref_speed.y = 0;

	/* compute speed error    */
	VECT2_DIFF(of_hover_speed_err, ref_speed, vel_from_GPS);
	/* saturate it               */
	VECT2_STRIM(of_hover_speed_err, -MAX_SPEED_ERR, MAX_SPEED_ERR);

	if(optiVelOnly)
	{
		of_hover_pos_err.x = 0;
		of_hover_pos_err.y = 0;
	}

	/* run PID */
	of_hover_cmd_earth.x =
			((GUIDANCE_H_PGAIN * of_hover_pos_err.x) >> (INT32_POS_FRAC - GH_GAIN_SCALE)) +
			((GUIDANCE_H_DGAIN * (of_hover_speed_err.x >> 2)) >> (INT32_SPEED_FRAC - GH_GAIN_SCALE - 2));
	of_hover_cmd_earth.y =
			((GUIDANCE_H_PGAIN * of_hover_pos_err.y) >> (INT32_POS_FRAC - GH_GAIN_SCALE)) +
			((GUIDANCE_H_DGAIN * (of_hover_speed_err.y >> 2)) >> (INT32_SPEED_FRAC - GH_GAIN_SCALE - 2));

	/* trim max bank angle from PD */
	VECT2_STRIM(of_hover_cmd_earth, -traj_max_bank, traj_max_bank);

	of_hover_trim_att_integrator.x += (GUIDANCE_H_IGAIN * of_hover_cmd_earth.x);
	of_hover_trim_att_integrator.y += (GUIDANCE_H_IGAIN * of_hover_cmd_earth.y);
	/* saturate it  */
	VECT2_STRIM(of_hover_trim_att_integrator, -(traj_max_bank << (INT32_ANGLE_FRAC + GH_GAIN_SCALE * 2)),
			(traj_max_bank << (INT32_ANGLE_FRAC + GH_GAIN_SCALE * 2)));

	/* add it to the command */
	of_hover_cmd_earth.x += (of_hover_trim_att_integrator.x >> (INT32_ANGLE_FRAC + GH_GAIN_SCALE * 2));
	of_hover_cmd_earth.y += (of_hover_trim_att_integrator.y >> (INT32_ANGLE_FRAC + GH_GAIN_SCALE * 2));

	VECT2_STRIM(of_hover_cmd_earth, -total_max_bank, total_max_bank);

	// Compute Angle Setpoints - Taken from Stab_att_quat
	int32_t s_psi, c_psi;
	PPRZ_ITRIG_SIN(s_psi, psi);
	PPRZ_ITRIG_COS(c_psi, psi);

	if(phi)
	{
		opti_sp_eu->phi = (-s_psi * of_hover_cmd_earth.x + c_psi * of_hover_cmd_earth.y) >> INT32_TRIG_FRAC;
	}
	if(theta)
	{
		opti_sp_eu->theta= -(c_psi * of_hover_cmd_earth.x + s_psi * of_hover_cmd_earth.y) >> INT32_TRIG_FRAC;
	}
}
