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
 * @file vl6180x.c
 * @copyright Copyright 2022 Jean-Marcel Herzog. This project is released under the MIT license.
 * @date 11.05.2022
 * @version 1
*/

#include "vl6180x.h"

void DistanceSensor_SetRegister(tDistanceSensor *sensor, uint16_t registerAddr, uint8_t data);

void DistanceSensor_SetRegister16bit(tDistanceSensor *sensor, uint16_t registerAddr, uint16_t data);

void DistanceSensor_GetRegister(tDistanceSensor *sensor, uint16_t registerAddr);

void DistanceSensor_GetRegister16bit(tDistanceSensor *sensor, uint16_t registerAddr);


/**
 * @brief Initialize VL6180X distance sensor
 *
 * Write default registers to sensor.
 * This follows according the official manufacture library.
 *
 * @param sensor Distance sensor instance - store initial values
 *
 * @param i2c STM_HAL instance to i2c hardware
 *
 * @param address I2C address of the target sensor to initialize
 *
 * @return 0x00 := NO ERROR | 0x01 := UNREFERENCED INSTANCE
 */
uint8_t DistanceSensor_Init(tDistanceSensor *sensor, I2C_HandleTypeDef *i2c, uint8_t address)
{
	if (sensor == NULL || i2c == NULL)
		return 0x01;

	sensor->hi2cx		= i2c;
	sensor->address		= address;
	sensor->data		= 0x00;
	sensor->data_16		= 0x00;
	sensor->distance	= 0x00;
	sensor->trig		= 0x00;

	/* REGISTER_TUNING_SR03_270514_CustomerView.txt */
	DistanceSensor_SetRegister(sensor, VL6180X_SYSTEM_FRESH_OUT_OF_RESET, 0x01);
	DistanceSensor_SetRegister(sensor,0x0207, 0x01);
	DistanceSensor_SetRegister(sensor,0x0208, 0x01);
	DistanceSensor_SetRegister(sensor,0x0096, 0x00);
	DistanceSensor_SetRegister(sensor,0x0097, 0xfd);
	DistanceSensor_SetRegister(sensor,0x00e3, 0x00);
	DistanceSensor_SetRegister(sensor,0x00e4, 0x04);
	DistanceSensor_SetRegister(sensor,0x00e5, 0x02);
	DistanceSensor_SetRegister(sensor,0x00e6, 0x01);
	DistanceSensor_SetRegister(sensor,0x00e7, 0x03);
	DistanceSensor_SetRegister(sensor,0x00f5, 0x02);
	DistanceSensor_SetRegister(sensor,0x00d9, 0x05);
	DistanceSensor_SetRegister(sensor,0x00db, 0xce);
	DistanceSensor_SetRegister(sensor,0x00dc, 0x03);
	DistanceSensor_SetRegister(sensor,0x00dd, 0xf8);
	DistanceSensor_SetRegister(sensor,0x009f, 0x00);
	DistanceSensor_SetRegister(sensor,0x00a3, 0x3c);
	DistanceSensor_SetRegister(sensor,0x00b7, 0x00);
	DistanceSensor_SetRegister(sensor,0x00bb, 0x3c);
	DistanceSensor_SetRegister(sensor,0x00b2, 0x09);
	DistanceSensor_SetRegister(sensor,0x00ca, 0x09);
	DistanceSensor_SetRegister(sensor,0x0198, 0x01);
	DistanceSensor_SetRegister(sensor,0x01b0, 0x17);
	DistanceSensor_SetRegister(sensor,0x01ad, 0x00);
	DistanceSensor_SetRegister(sensor,0x00ff, 0x05);
	DistanceSensor_SetRegister(sensor,0x0100, 0x05);
	DistanceSensor_SetRegister(sensor,0x0199, 0x05);
	DistanceSensor_SetRegister(sensor,0x01a6, 0x1b);
	DistanceSensor_SetRegister(sensor,0x01ac, 0x3e);
	DistanceSensor_SetRegister(sensor,0x01a7, 0x1f);
	DistanceSensor_SetRegister(sensor,0x0030, 0x00);

	/* Recommended : Public registers - See data sheet for more detail */
	DistanceSensor_SetRegister(sensor, 0x0011, 0x10); /* Enables polling for New Sample ready when measurement completes */
	DistanceSensor_SetRegister(sensor, 0x010a, 0x30); /* Set the averaging sample period (compromise between lower noise and increased execution time) */
	DistanceSensor_SetRegister(sensor, 0x003f, 0x46); /* Sets the light and dark gain (upper nibble). Dark gain should not be changed.*/
	DistanceSensor_SetRegister(sensor, 0x0031, 0xFF); /* sets the # of range measurements after which auto calibration of system is performed */
	DistanceSensor_SetRegister(sensor, 0x0040, 0x63); /* Set ALS integration time to 100ms */
	DistanceSensor_SetRegister(sensor, 0x002e, 0x01); /* perform a single temperature calibration of the ranging sensor */

	/* Optional: Public registers - See data sheet for more detail */
	DistanceSensor_SetRegister(sensor, 0x001b, 0x09); /* Set default ranging inter-measurement period to 100ms */
	DistanceSensor_SetRegister(sensor, 0x003e, 0x31); /* Set default ALS inter-measurement period to 500ms */
	DistanceSensor_SetRegister(sensor, 0x0014, 0x24); /* Configures interrupt on New sample ready */

	return 0x00;
}

/**
 * @brief Change I2C address
 *
 * @param sensor Distance sensor data struct instance
 *
 * @param new_addr New I2C address
 *
 * @return 0x00 := NO ERROR | 0x01 := UNREFERENCED INSTANCE | 0x02 := INVALID I2C ADDRESS
 */
uint8_t DistanceSensor_ChangeAddress(tDistanceSensor *sensor, uint8_t new_addr)
{
	if (sensor == NULL)
		return 0x01;

	if (sensor->address == new_addr) return 0x00;
	if (new_addr > 127) return 0x02;

	DistanceSensor_SetRegister(sensor, VL6180X_I2C_SLAVE_DEVICE_ADDRESS, new_addr);
	sensor->address = new_addr << 1;
	DistanceSensor_SetRegister(sensor, VL6180X_I2C_SLAVE_DEVICE_ADDRESS, 0x07);

	return 0x00;
}

/**
 * @brief Get Sensor Model informations
 *
 * @param sensor Distance sensor data struct instance
 *
 * @param identification
 *
 * @return 0x00
 */
uint8_t DistanceSensor_GetIdentification(tDistanceSensor *sensor, tDistanceSensorIdentification *identification)
{
	DistanceSensor_GetRegister(sensor, VL6180X_IDENTIFICATION_MODEL_ID);
	identification->model = sensor->data;

	DistanceSensor_GetRegister(sensor, VL6180X_IDENTIFICATION_MODEL_REV_MAJOR);
	identification->model_rev_major = sensor->data;

	DistanceSensor_GetRegister(sensor, VL6180X_IDENTIFICATION_MODEL_REV_MINOR);
	identification->model_rev_minor = sensor->data;

	DistanceSensor_GetRegister(sensor, VL6180X_IDENTIFICATION_MODULE_REV_MAJOR);
	identification->module_rev_major = sensor->data;
	DistanceSensor_GetRegister(sensor, VL6180X_IDENTIFICATION_MODULE_REV_MINOR);
	identification->module_rev_minor = sensor->data;

	DistanceSensor_GetRegister16bit(sensor, VL6180X_IDENTIFICATION_DATE_HI);
	identification->date = sensor->data_16;
	DistanceSensor_GetRegister16bit(sensor, VL6180X_IDENTIFICATION_TIME_1);
	identification->date = sensor->data_16;

	return 0x00;
}

/**
 * @brief Starting sensor in continous mode
 *
 * @param sensor Distance sensor data struct instance
 *
 * @return 0x00
 */
uint8_t DistanceSensor_StartContinousMeasurements(tDistanceSensor *sensor)
{
	DistanceSensor_SetRegister(sensor, VL6180X_SYSALS_INTERMEASUREMENT_PERIOD, 0x00);
	DistanceSensor_SetRegister(sensor, VL6180X_SYSRANGE_MAX_CONVERGENCE_TIME, 0x01);
	HAL_Delay(10);
	DistanceSensor_SetRegister(sensor, VL6180X_SYSRANGE_START, 0x03);
	HAL_Delay(10);

	return 0x00;
}

/**
 * @brief Trigger new distance measurement
 *
 * @param sensor Distance sensor data struct instance
 */
uint8_t DistanceSensor_Trigger(tDistanceSensor *sensor)
{
	DistanceSensor_SetRegister(sensor, VL6180X_SYSRANGE_START, 0x01);
	sensor->trig = 0x01;
	return 0x00;
}

/**
 * @brief Response for measred distance
 *
 * This function read the response register and clear the interrupt on the sensor.
 *
 * @param sensor Distance sensor data struct instance
 *
 * @return 0x00
 */
uint8_t DistanceSensor_TrigResponse(tDistanceSensor *sensor)
{
	DistanceSensor_GetRegister(sensor, VL6180X_RESULT_RANGE_VAL);
	sensor->distance = sensor->data;
	DistanceSensor_SetRegister(sensor, VL6180X_SYSTEM_INTERRUPT_CLEAR, 0x07);
	sensor->trig = 0x00;
	return 0x00;
}

/**
 * @brief Start Measurements in Continuous Mode
 *
 * @param sensor Distance sensor data struct instance
 *
 * @return 0x00
 */
uint8_t DistanceSensor_GetDistanceContinous(tDistanceSensor *sensor)
{
	DistanceSensor_GetRegister(sensor, VL6180X_RESULT_RANGE_VAL);
	sensor->distance = sensor->data;

	return 0x00;
}

/**
 * @brief Read latest distance
 *
 * @param sensor Distance sensor data struct instance
 *
 * @param sensor Distance sensor
 *
 * @return 0x00
 */
uint8_t DistanceSensor_GetDistance(tDistanceSensor *sensor)
{
	DistanceSensor_SetRegister(sensor, VL6180X_SYSRANGE_START, 0x01);
	HAL_Delay(10);
	DistanceSensor_GetRegister(sensor, VL6180X_RESULT_RANGE_VAL);
	sensor->distance = sensor->data;
	DistanceSensor_SetRegister(sensor, VL6180X_SYSTEM_INTERRUPT_CLEAR, 0x07);

	return 0x00;
}

/**
 * @brief Write 8-Bit data into sensor register
 *
 * @param sensor Distance sensor data struct instance
 *
 * @param registerAddr Sensor register address
 *
 * @param data Values to write into register
 */
void DistanceSensor_SetRegister(tDistanceSensor *sensor, uint16_t registerAddr, uint8_t data)
{
	uint8_t buffer[3];
	buffer[0] = (registerAddr >> 8) & 0xFF;
	buffer[1] = (registerAddr & 0xFF);
	buffer[2] = data & 0xFF;
	HAL_I2C_Master_Transmit(sensor->hi2cx, sensor->address, buffer, 3, 300);
}

/**
 * @brief Write 16-Bit data into sensor register
 *
 * @param sensor Distance sensor data struct instance
 *
 * @param registerAddr Sensor register address
 *
 * @param data Values to write into register
 */
void DistanceSensor_SetRegister16bit(tDistanceSensor *sensor, uint16_t registerAddr, uint16_t data)
{
	uint8_t buffer[4];
	buffer[0] = (registerAddr >> 8) & 0xFF;
	buffer[1] = (registerAddr & 0xFF);
	buffer[2] = (data >> 8) & 0xFF;
	buffer[3] = data & 0xFF;
	HAL_I2C_Master_Transmit(sensor->hi2cx, sensor->address, buffer, 3, 300);
}

/**
 * @brief Read 8-Bit data from sensor register
 *
 * The data from the sensor register is placed into the sensor datastruct (sensor.data)
 *
 * @param sensor Distance sensor data struct instance
 *
 * @param registerAddr Sensor register address
 */
void DistanceSensor_GetRegister(tDistanceSensor *sensor, uint16_t registerAddr)
{
	uint8_t buffer[2];
	uint8_t data;

	buffer[0] = (registerAddr >> 8) & 0xFF;
	buffer[1] = (registerAddr & 0xFF);

	HAL_I2C_Master_Transmit(sensor->hi2cx, sensor->address, buffer, 2, 300);
	HAL_I2C_Master_Receive(sensor->hi2cx, sensor->address, &data, 1, 300);

	sensor->data = data;
}

/**
 * @brief Read 16-Bit data from sensor register
 *
 * The data from the sensor register is placed into the sensor datastruct (sensor.data)
 *
 * @param sensor Distance sensor data struct instance
 *
 * @param registerAddr Sensor register address
 */
void DistanceSensor_GetRegister16bit(tDistanceSensor *sensor, uint16_t registerAddr)
{
	uint8_t buffer[2];
	uint8_t data[2];

	buffer[0] = (registerAddr >> 8) & 0xFF;
	buffer[1] = registerAddr & 0xFF;

	HAL_I2C_Master_Transmit(sensor->hi2cx, sensor->address, buffer, 2, 300);
	HAL_I2C_Master_Receive(sensor->hi2cx, sensor->address, data, 2, 300);

	sensor->data_16 = ((uint16_t)data[0] << 8) | (uint16_t)data[1];

}
