/*
 * machine_parameter.h
 *
 *  Created on: May 10, 2022
 *      Author: Jean-Marcel Herzog
 */

#ifndef INC_MACHINE_PARAMETER_H_
#define INC_MACHINE_PARAMETER_H_

/*
 * Software Simulation Mode
 */
#define SIM_MODE

/*
 * Measurements per row
 */
#define N_MEASUREMENTS	100

/*
 * Default Parameters for scanner table
 */
#define TABLE_STEPS_PER_ROTATION 12800
#define TABLE_DEG_PER_STEP (360.0 / TABLE_STEPS_PER_ROTATION)
#define TABLE_MIN_POS 0.0
#define TABLE_MAX_POS 360.0

/*
 * Default Parameters for scanner lift
 */
#define LIFT_STEPS_PER_MM 1024
#define LIFT_MAX_POS 90.0
#define LIFT_MIN_POS 0.0

/*
 * Default Parameters for distance sensor A
 */
#define SENSOR_A_OFFSET_X 0
#define SENSOR_A_OFFSET_Y 0

/*
 * Default Parameters for distance sensor B
 */
#define SENSOR_B_OFFSET_X 0
#define SENSOR_B_OFFSET_Y 0

/*
 * SCANNER STATES
 */
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
