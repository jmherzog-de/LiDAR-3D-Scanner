/**
* Copyright 2022 Jean-Marcel Herzog
*
* Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
* associated documentation files (the "Software"), to deal in the Software without restriction,
* including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense
* and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do
* so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in all copies or
* substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
* INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
* PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
* COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN
* AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
* WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

/**
 * @author: Herzog, Jean-Marcel
 * @file motion_profile.h
 * @copyright Copyright 2022 Jean-Marcel Herzog. This project is released under the MIT license.
 * @date 11.05.2022
 * @version 1
*/

#ifndef INC_MOTION_PROFILE_H_
#define INC_MOTION_PROFILE_H_

#include <stdint.h>

typedef struct {
	uint8_t enabled;	/**< Motion Profile enabled flag - If enabled := Motor running */
	uint32_t N;			/**< Steps to make for the desired movement. */
	uint32_t N_a;		/**< Steps to make while stepper accelerating. */
	uint32_t N_v;		/**< Steps to make while stepper has constant speed. */
	uint32_t N_d;		/**< Steps to make while stepper is decelerating. */
	int32_t np;			/**< internal variable to store moved steps. */
	double v_sf;		/**< internal variable to store current calculated velocity. */
	double A;			/**< Acceleration. */
	double D;			/**< Deceleration. */
	double alpha;		/**< Internal variable to store qutient between full turn and steps. */
	float T;			/**< Period Time to perform the desired movement. */
	float t_a;			/**< Calculated time for acceleration. */
	float t_v;			/**< Calculated time for constant speed. */
	float t_d;			/**< Calculated time for decelerating movement. */
	double v;			/**< Current speed of the stepper motor. */
	float v_s;			/**< Initial velocity for first step. */
	float v_m;			/**< Maximum velocity variable.*/
	float a;			/**< Current acceleration of the stepper motor. */
	float a_m;			/**< Maximum acceleration variable. */
	float s;			/**< Internal variable to save distance within one movement. */
	float pos;			/**< Current position of the stepper (saved over multiple movements).  */
	double p;			/**< Internal variable to store position for the current movement. */
	double dp;			/**< Internal variable to save delta position. */
} tMotionProfile;

uint8_t reset_motion_profile(tMotionProfile *profile, float f);

uint8_t generate_motion_profile(tMotionProfile *profile, float s, float v_s, float a_m, float v_m, uint32_t k, float f);

#endif /* INC_MOTION_PROFILE_H_ */
