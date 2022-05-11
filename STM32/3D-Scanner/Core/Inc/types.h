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
 * @file types.h
 * @copyright Copyright 2022 Jean-Marcel Herzog. This project is released under the MIT license.
 * @date 11.05.2022
 * @version 1
*/

#ifndef INC_TYPES_H_
#define INC_TYPES_H_

#include "motion_profile.h"
#include "vl6180x.h"

typedef struct {

	tMotionProfile lift_profile;	/**< Linear axis motion profile data. */
	tMotionProfile table_profile;	/**< Table motion profile data. */

	tDistanceSensor sensor_a;		/**< Sensor A data. */
	tDistanceSensor sensor_b;		/**< Sensor B data. */

	uint8_t lift_dir;				/**< Linear axis current selected direction 0x00:= UP 0x01:=DOWN */
	uint8_t state;					/**< Current state of the 3D-Scanner. */
	uint8_t scan_running;			/**< Scan currently running flag. */
	uint8_t lift_homed;				/**< Set when lift reached end-stop switch. */

} tScanner;

typedef struct {
	uint8_t distance;				/**< Measured distance. */
	float position;					/**< Table position [°]. */
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
