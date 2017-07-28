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
// With optitrack set to:    NAV
#define GUIDANCE_V_MODE_MODULE_SETTING GUIDANCE_V_MODE_NAV
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
float nominalthrust;
float magicfactorX;
float magicfactorY;
bool oscphi;
bool osctheta;
float FFfactor;
float FFtime;
bool performFFtakeoff;
bool performXY;
bool performZ;
float covZthreshold;
uint32_t covXthreshold;
uint32_t covYthreshold;
float startPusedZ;
float rampZ;

float startPusedY;
float rampY;
float startPusedX;
float rampX;

float testigainZ;

struct OpticalFlowTitus {
	float lp_factor;              ///< low-pass factor in [0,1], with 0 purely using the current measurement
	float divergence_setpoint;    ///< setpoint for constant divergence approach
	float ventralflow_setpoint;    ///< setpoint for constant ventral flow approach
	int delay_steps;              ///< number of delay steps for div past

	float sum_errX;                ///< integration of the error for I-gain in X axis
	float sum_errY;                ///< integration of the error for I-gain in Y axis
	float sum_errZ;                ///< integration of the error for I-gain in Z axis
};

struct OpticalFlowLanding {
  float agl;                    ///< agl = height from sonar (only used when using "fake" divergence)
  float agl_lp;                 ///< low-pass version of agl
  float lp_factor;              ///< low-pass factor in [0,1], with 0 purely using the current measurement
  float vel;                    ///< vertical velocity as determined with sonar (only used when using "fake" divergence)
  float divergence_setpoint;    ///< setpoint for constant divergence approach
  float pgain;                  ///< P-gain for constant divergence control (from divergence error to thrust)
  float igain;                  ///< I-gain for constant divergence control
  float dgain;                  ///< D-gain for constant divergence control
  float sum_err;                ///< integration of the error for I-gain
  float nominal_thrust;         ///< nominal thrust around which the PID-control operates
  int VISION_METHOD;            ///< whether to use vision (1) or Optitrack / sonar (0)
  int CONTROL_METHOD;           ///< type of divergence control: 0 = fixed gain, 1 = adaptive gain
  float cov_set_point;          ///< for adaptive gain control, setpoint of the covariance (oscillations)
  float cov_limit;              ///< for fixed gain control, what is the cov limit triggering the landing
  float pgain_adaptive;         ///< P-gain for adaptive gain control
  float igain_adaptive;         ///< I-gain for adaptive gain control
  float dgain_adaptive;         ///< D-gain for adaptive gain control
  int COV_METHOD;               ///< method to calculate the covariance: between thrust and div (0) or div and div past (1)
  int delay_steps;              ///< number of delay steps for div past
};


#endif


