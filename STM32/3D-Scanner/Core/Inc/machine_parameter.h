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
 * @file machine_parameter.h
 * @copyright Copyright 2022 Jean-Marcel Herzog. This project is released under the MIT license.
 * @date 11.05.2022
 * @version 1
*/

#ifndef INC_MACHINE_PARAMETER_H_
#define INC_MACHINE_PARAMETER_H_

//
//Software Simulation Mode
//
#define SIM_MODE

//
// Measurements per row
///
#define N_MEASUREMENTS	100

//
// Default Parameters for scanner table
//
#define TABLE_STEPS_PER_ROTATION 12800
#define TABLE_DEG_PER_STEP (360.0 / TABLE_STEPS_PER_ROTATION)
#define TABLE_MIN_POS 0.0
#define TABLE_MAX_POS 360.0

//
// Default Parameters for scanner lift
//
#define LIFT_STEPS_PER_MM 1024
#define LIFT_MAX_POS 90.0
#define LIFT_MIN_POS 0.0

//
// Default Parameters for distance sensor A
//
#define SENSOR_A_OFFSET_X 0
#define SENSOR_A_OFFSET_Y 0

//
// Default Parameters for distance sensor B
//
#define SENSOR_B_OFFSET_X 0
#define SENSOR_B_OFFSET_Y 0

//
// SCANNER STATES
//
#define STATE_INITIALIZING			0x00
#define STATE_INITIALIZED			0x01
#define STATE_READY_TO_SCAN			0x02
#define STATE_MOVE_LIFT				0x03
#define STATE_MOVE_TABLE			0x04
#define STATE_SCAN_ROW				0x05
#define STATE_TRANSFER_DATA			0x06
#define STATE_CALC_POINTS			0x07
#define STATE_CALC_STL				0x08
#define STATE_HOME_LIFT				0x09

#endif /* INC_MACHINE_PARAMETER_H_ */
