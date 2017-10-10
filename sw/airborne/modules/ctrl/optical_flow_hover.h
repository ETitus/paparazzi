/*
 * Copyright (C) 2016
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

#ifndef OPTICAL_FLOW_HOVER_H_
#define OPTICAL_FLOW_HOVER_H_

#include "std.h"


// Without optitrack set to: GUIDANCE_V/H_MODE_ATTITUDE
// With optitrack set to: GUIDANCE_V/H_MODE_NAV
// To use the Optical Flow Hover module use GUIDANCE_V/H_MODE_MODULE

#define GUIDANCE_H_MODE_MODULE_SETTING GUIDANCE_H_MODE_MODULE
//#define GUIDANCE_H_MODE_MODULE_SETTING GUIDANCE_H_MODE_NAV

#define GUIDANCE_V_MODE_MODULE_SETTING GUIDANCE_V_MODE_MODULE
//#define GUIDANCE_V_MODE_MODULE_SETTING GUIDANCE_V_MODE_NAV

bool oscphi;
bool osctheta;
bool derotated;

struct OpticalFlowHover {
	float pgainX;                  ///< P-gain for constant divergence control (from divergence error to thrust)
	float igainX;                  ///< I-gain for constant divergence control
	float dgainX;                  ///< D-gain for constant divergence control

	float pgainY;                  ///< P-gain for constant divergence control (from divergence error to thrust)
	float igainY;                  ///< I-gain for constant divergence control
	float dgainY;                  ///< D-gain for constant divergence control

	float pgainZ;                  ///< P-gain for constant divergence control (from divergence error to thrust)
	float igainZ;                  ///< I-gain for constant divergence control
	float dgainZ;                  ///< D-gain for constant divergence control

	float rampXY;				   ///< The ramp pused is increased with per dt
	float rampZ;				   ///< The ramp pused is increased with per dt

	float divergence;              ///< Divergence estimate
	float flowX;			       ///< Flow estimate in X direction
	float flowY;			       ///< Flow estimate in Y direction

	float previous_errX;           ///< Previous divergence tracking error
	float sum_errX;                ///< integration of the error for I-gain
	float d_errX;                  ///< difference of error for the D-gain

	float previous_errY;           ///< Previous divergence tracking error
	float sum_errY;                ///< integration of the error for I-gain
	float d_errY;                  ///< difference of error for the D-gain

	float previous_errZ;           ///< Previous divergence tracking error
	float sum_errZ;                ///< integration of the error for I-gain
	float d_errZ;                  ///< difference of error for the D-gain

	float nominal_thrust;          ///< nominal thrust around which the PID-control operates


	float divergence_setpoint;     ///< setpoint for constant divergence approach
	float covDiv_set_point;        ///< for adaptive gain control, setpoint of the covariance (oscillations)

	float flow_setpoint;         ///< setpoint for constant divergence approach
	float covFlow_set_point;       ///< for adaptive gain control, setpoint of the covariance (oscillations)

	float reduction_factorXY;      ///< Reduce the XY gains by this factor when oscillating
	float reduction_factorZ;       ///< Reduce the Z gains by this factor when oscillating

	uint16_t window_size;
	uint16_t delay_steps;
	float lp_const;                ///< low-pass filter constant
	uint32_t COV_METHOD;           ///< method to calculate the covariance: between thrust and div (0) or div and div past (1)
	bool ofmethode;				   ///< Select which Divergence to take, size_divergence or regular_divergence
};

extern struct OpticalFlowHover of_hover_ctrl;

// The module functions
extern void optical_flow_hover_init(void);
extern void optical_flow_hover_start(void);
extern void optical_flow_hover_periodic(void);
extern void optical_flow_hover_stop(void);

// Implement own Vertical loops
extern void guidance_v_module_init(void);
extern void guidance_v_module_enter(void);
extern void guidance_v_module_run(bool in_flight);

// Implement own Horizontal loops
extern void guidance_h_module_init(void);
extern void guidance_h_module_enter(void);
extern void guidance_h_module_run(bool in_flight);

#endif /* OPTICAL_FLOW_LANDING_H_ */
