/*
 * motion_profile.h
 *
 *  Created on: May 10, 2022
 *      Author: Jean-Marcel
 */

#ifndef INC_MOTION_PROFILE_H_
#define INC_MOTION_PROFILE_H_

#include <stdint.h>

typedef struct {
	uint8_t enabled;
	uint32_t N;
	uint32_t N_a;
	uint32_t N_v;
	uint32_t N_d;
	int32_t np;
	double v_sf;
	double A;
	double D;
	double alpha;
	float T;
	float t_a;
	float t_v;
	float t_d;
	double v;
	float v_s;
	float v_m;
	float a;
	float a_m;
	float s;
	float pos;
	double p;
	double dp;
} tMotionProfile;

uint8_t reset_motion_profile(tMotionProfile *profile, float f);

uint8_t generate_motion_profile(tMotionProfile *profile, float s, float v_s, float a_m, float v_m, uint32_t k, float f);

#endif /* INC_MOTION_PROFILE_H_ */
