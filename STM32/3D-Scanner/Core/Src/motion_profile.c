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
 * @file motion_profile.c
 * @copyright Copyright 2022 Jean-Marcel Herzog. This project is released under the MIT license.
 * @date 11.05.2022
 * @version 1
*/

#include <stddef.h>
#include <math.h>

#include "types.h"
#include "motion_profile.h"

/*
 * @brief Reset Motion Profile Parameters.
 *
 * @param profile Pointer to motion profile structure which should be cleared.
 *
 * @param f Timer Sampling Rate.
 *
 * @return 0x00 := NO_ERROR
 */
uint8_t reset_motion_profile(tMotionProfile *profile, float f)
{
	profile->A		= profile->a_m / profile->alpha / f / f;
	profile->D		= -1.0 * profile->A;
	profile->v_sf	= profile->v_s / profile->alpha / f;
	profile->v		= profile->v_sf;
	profile->p		= 0.0;
	profile->np		= 0;
	profile->dp		= 0.0;

	return 0x00;
}

/*
 * @brief Calculate motion profile for stepper motor with the given parameters.
 *
 * This function create either a triangular or rectangular speed profile. It is
 * depending on the input parameters.
 *
 * @param profile Pointer to motion profile structure
 *
 * @param s Angle to turn [°]
 *
 * @param v_s Initial velocity [°/s]
 *
 * @param a_m Maximum acceleration [°/s^2]
 *
 * @param v_m Maximum velocity [°/s]
 *
 * @param k Steps for one full rotation
 *
 * @param f Timer Sampling Rate
 *
 * @return 0x00 := NO_ERROR | 0x01 := INVALID_PROFILE_POINTER | 0x02 := Incorrect params | 0x03 := Maximum velocity lower initial velocity
 *
 */
uint8_t generate_motion_profile(tMotionProfile *profile, float s, float v_s, float a_m, float v_m, uint32_t k, float f)
{
	if (profile == NULL)
		return 0x01;
	if (s <= 0.0 || v_s <= 0.0 || a_m <= 0.0 || v_m <= 0.0 || k == 0 || f <= 0.0)
		return 0x02;
	if (v_m < v_s)
		return 0x03;

	profile->a_m	= a_m;
	profile->v_s	= v_s;
	profile->v_m	= v_m;
	profile->s		= s;

	// Calculate parameters assuming triangular speed profile.
	profile->alpha	= 360.0 / (float)k;
	profile->N		= profile->s / profile->alpha;
	profile->a		= profile->a_m;
	profile->t_v	= 0.0;
	profile->T		= -2.0 * profile->v_s / profile->a;
	profile->T		= profile->T + sqrtf( (profile->v_s * profile->v_s * 2.0 / profile->a / profile->a) + (4.0 * profile->s / profile->a));
	profile->t_a	= profile->T / 2.0;
	profile->t_d	= profile->T - profile->t_a;
	profile->v		= profile->v_s + ( profile->a * profile->t_a);

	if (profile->v > profile->v_m)
	{
		// trapezoidal speed profile
		profile->v		= profile->v_m;
		profile->t_a	= (profile->v - profile->v_s) / profile->a;
		float T_ad		= profile->t_a * 2.0;
		profile->t_d	= T_ad - profile->t_a;
		float s_ad		= (profile->v_s * T_ad) + ( profile->a * T_ad * T_ad * 0.25);
		profile->t_v	= (profile->s - s_ad) / profile->v;
		profile->T		= T_ad + profile->t_v;
		uint32_t N_ad	= s_ad / profile->alpha;
		profile->N_a	= N_ad * 0.5;
		profile->N_d	= N_ad - profile->N_a;
		profile->N_v	= profile->N - N_ad;
	}
	else
	{
		// triangular speed profile
		profile->N_a	= profile->N * 0.5;
		profile->N_d	= profile->N - profile->N_a;
		profile->N_v	= 0;

	}

	profile->A		= profile->a_m / profile->alpha / f / f;
	profile->D		= -1.0 * profile->A;
	profile->v_sf	= profile->v_s / profile->alpha / f;
	profile->v		= profile->v_sf;
	profile->p		= 0.0;
	profile->np		= 0;
	profile->dp		= 0.0;

	return 0x00;
}
