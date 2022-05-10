/*
 * types.h
 *
 *  Created on: May 10, 2022
 *      Author: Jean-Marcel
 */

#ifndef INC_TYPES_H_
#define INC_TYPES_H_

#include "motion_profile.h"
#include "vl6180x.h"

typedef struct {

	tMotionProfile lift_profile;
	tMotionProfile table_profile;

	tDistanceSensor sensor_a;
	tDistanceSensor sensor_b;

	uint8_t lift_dir;
	uint8_t state;
	uint8_t scan_running;
	uint8_t lift_homed;

} tScanner;

typedef struct {
	uint8_t distance;
	float position;
} tMeasurement;

typedef struct {
	float x;
	float y;
	float z;
} tPoint;

typedef struct {
	tPoint *p1;
	tPoint *p2;
	tPoint *p3;
	tPoint *n;
} tVertex;

#endif /* INC_TYPES_H_ */
