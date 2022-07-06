/* USER CODE BEGIN Header */

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
 * @file main.c
 * @copyright Copyright 2022 Jean-Marcel Herzog. This project is released under the MIT license.
 * @date 11.05.2022
 * @version 1
*/

/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "machine_parameter.h"
#include "vl6180x.h"
#include "types.h"
#include <math.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
tScanner scanner;							/**< Scanner instance - contains all informations about the scanner. */
size_t point_index;							/**< Calculated point buffer index. */
tPoint points[10000];						/**< Buffer for calculated points. */
size_t measurement_index;					/**< Measured distances buffer index. */
tMeasurement measurements[N_MEASUREMENTS];	/**< Buffer for measured distances for one table turn. */
uint8_t tim2_activated;						/**< Flag for TIMER2 Interrupt - 0x01 := Interrupt activated 0x00 := Interrupt deactivated */
uint8_t tim3_activated;						/**< Flag for TIMER3 Interrupt - 0x01 := Interrupt activated 0x00 := Interrupt deactivated */
uint8_t tim4_activated;						/**< Flag for TIMER4 interrupt - 0x01 := Interrupt activated 0x00 := Interrupt deactivated */
uint8_t Z;									/**< Scanned rows counter variable. */
uint8_t i;									/**< Global index variable used by for-loops */
uint8_t invalid_measurements;				/**< Flag variable - 0x01 := Invalid measurements detected 0x00 := All measurements of last scanned row valid */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */
int16_t convert_float_to_int16(float *value);
uint8_t calculate_point(tPoint *p, size_t measurement_index);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

	//----------------------------------//
	// Initialize Scanner Object.		//
	//----------------------------------//
	scanner.state			= STATE_INITIALIZING;
	scanner.scan_running	= 0x00;
	scanner.lift_homed		= 0x00;
	scanner.lift_dir		= 0x00;
	point_index				= 0;
	measurement_index		= 0;
	tim3_activated			= 0;
	tim2_activated			= 0;
	invalid_measurements	= 0;

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

  uint8_t sensor_addr = 1;
  uint8_t ret = 0x00;
  while ( sensor_addr <= 255){
	  ret  = HAL_I2C_IsDeviceReady(&hi2c1, sensor_addr++, 3, 5);
	  if (ret == HAL_OK)
		  break;
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

		/*--------------------------------------------
		 * STATE: INITIALIZING
		 * --------------------------------------------*/
		if (scanner.state == STATE_INITIALIZING) {

			// Disable both stepper motors.
			HAL_GPIO_WritePin(TABLE_EN_GPIO_Port, TABLE_EN_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(LIFT_EN_GPIO_Port, LIFT_EN_Pin, GPIO_PIN_SET);

			// Initialize Distance Sensor
			DistanceSensor_Init(&scanner.sensor_a, &hi2c1, sensor_addr);

			// Set direction of lift to upwards
			scanner.lift_dir			= 0x01;
			scanner.lift_profile.pos	= LIFT_MIN_POS;

			// Set direction for table
			scanner.table_profile.pos = 0.0;
			HAL_GPIO_WritePin(TABLE_DIRECTION_GPIO_Port, TABLE_DIRECTION_Pin, GPIO_PIN_SET);

			// Start interrupt for status LED on evaluation board
			if (!tim3_activated)
			{
				tim3_activated = 0x01;
				HAL_TIM_Base_Start_IT(&htim3);
			}

			// Start interrupt for distance measurements
			if (!tim4_activated)
			{
				tim4_activated = 0x01;
				HAL_TIM_Base_Start_IT(&htim4);
			}

			generate_motion_profile(&scanner.table_profile, 360.0, 0.5, 80.0, 90.0, TABLE_STEPS_PER_ROTATION, 10000.0);
			generate_motion_profile(&scanner.lift_profile, 720.0, 0.5 , 800.0, 800.0, LIFT_STEPS_PER_MM, 10000.0);

			scanner.state = STATE_INITIALIZED;
		}

		/*--------------------------------------------
		 * STATE: INITIALIZED
		 * --------------------------------------------*/
		if (scanner.state == STATE_INITIALIZED) {

			if (HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == 0x00 && HAL_GPIO_ReadPin(LIFT_ENDPOS_GPIO_Port, LIFT_ENDPOS_Pin) == 0x01)
				scanner.state = STATE_READY_TO_SCAN;
			else if (HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == 0x00)
				scanner.state = STATE_HOME_LIFT;
		}

		/*--------------------------------------------
		 * STATE: HOME LIFT
		 * --------------------------------------------*/
		if (scanner.state == STATE_HOME_LIFT) {

			// Initiate stepper movement.
			if(scanner.lift_profile.enabled == 0x00 && HAL_GPIO_ReadPin(LIFT_ENDPOS_GPIO_Port, LIFT_ENDPOS_Pin) == 0x01)
			{
				tim2_activated	= 0x00;
				HAL_GPIO_WritePin(LIFT_EN_GPIO_Port, LIFT_EN_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(LIFT_DIRECTION_GPIO_Port, LIFT_DIRECTION_Pin, GPIO_PIN_RESET);
				HAL_TIM_Base_Stop_IT(&htim2);
				scanner.lift_profile.pos = 0.0;
				scanner.state = STATE_READY_TO_SCAN;
			}

			// Stepper requested movement done.
			else if(scanner.lift_profile.enabled == 0x00)
			{
				scanner.lift_profile.enabled	= 0x01;
				tim2_activated					= 0x01;
				HAL_GPIO_WritePin(LIFT_DIRECTION_GPIO_Port, LIFT_DIRECTION_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(LIFT_EN_GPIO_Port, LIFT_EN_Pin, GPIO_PIN_RESET);
				HAL_TIM_Base_Start_IT(&htim2);
			}
		}

		/*--------------------------------------------
		 * STATE: READY TO SCAN
		 * --------------------------------------------*/
		if (scanner.state == STATE_READY_TO_SCAN) {

			// Deactivate Interrupt for Status LED
			if (tim3_activated == 0x01) {
				tim3_activated = 0x00;
				HAL_TIM_Base_Stop_IT(&htim3);
				HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
			}

			// Start Scan
			if ( HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == 0x00 )
			{
				scanner.scan_running = 0x01;
				scanner.state = STATE_MOVE_TABLE;
			}
		}

		/*--------------------------------------------
		 * STATE: STATE MOVE TABLE
		 * --------------------------------------------*/
		if (scanner.state == STATE_MOVE_TABLE) {

			// Initiate table stepper movement.
			if (tim2_activated == 0x00) {
				reset_motion_profile(&scanner.table_profile, 10000.0);

				// Initial scanned point at position 0.0°.
				measurements[measurement_index].distance = scanner.sensor_a.distance;
				measurements[measurement_index++].position = 0.0;

				scanner.table_profile.enabled = 0x01;
				tim2_activated = 0x01;
				HAL_GPIO_WritePin(TABLE_EN_GPIO_Port, TABLE_EN_Pin, GPIO_PIN_RESET);
				HAL_TIM_Base_Start_IT(&htim2);

			}

			// Stepper requested movement done
			if (scanner.table_profile.enabled == 0x00) {
				tim2_activated = 0x00;
				HAL_GPIO_WritePin(TABLE_EN_GPIO_Port, TABLE_EN_Pin, GPIO_PIN_SET);
				HAL_TIM_Base_Stop_IT(&htim2);

				if (scanner.lift_profile.pos >= 90.0)
					scanner.scan_running = 0x00;

				scanner.state = STATE_CALC_POINTS;

			} else if (scanner.scan_running == 0x01) { scanner.state = STATE_SCAN_ROW; }
		}

		/*--------------------------------------------
		 * STATE: STATE CALC AND TRANSFER POINTS
		 * --------------------------------------------*/
		if (scanner.state == STATE_CALC_POINTS) {

			uint8_t message[6] = { 0x00 };
			uint16_t temp;
			float end_val = -1.0;

			invalid_measurements = 0;
			for (i = 0; i < measurement_index; i++) {
				if (measurements[measurement_index].distance >= 100.0)
					invalid_measurements++;

				calculate_point(points + point_index, i);
				point_index++;
			}
			measurement_index = 0;

			// Abort current scan when invalid distances measured from sensor.
			if (invalid_measurements > 3) {
				scanner.scan_running = 0x00;
			} else {
				// Transfer points over UART
				for (i = point_index - N_MEASUREMENTS; i < point_index; i++) {
					temp = convert_float_to_int16(&(points[i].x));
					memcpy(message, &temp, sizeof(uint16_t));
					temp = convert_float_to_int16(&(points[i].y));
					memcpy(message + 2, &temp, sizeof(uint16_t));
					temp = convert_float_to_int16(&(points[i].z));
					memcpy(message + 4, &temp, sizeof(uint16_t));

					HAL_UART_Transmit(&huart2, message, sizeof(message), HAL_MAX_DELAY);
					HAL_Delay(10);
				}
			}

			if (scanner.scan_running == 0x01) {
				scanner.state = STATE_MOVE_LIFT;
			} else {
				// Transmit end of points message (-10, -100, -100)
				temp = convert_float_to_int16(&end_val);
				memcpy(message, &temp, sizeof(uint16_t));
				memcpy(message + 2, &temp, sizeof(uint16_t));
				memcpy(message + 4, &temp, sizeof(uint16_t));
				HAL_UART_Transmit(&huart2, message, sizeof(message), HAL_MAX_DELAY);
				HAL_Delay(10);

				scanner.state = STATE_CALC_STL;
			}
		}

		/*--------------------------------------------
		 * STATE: STATE CALC AND TRANSFER STL FILE
		 * --------------------------------------------*/
		if (scanner.state == STATE_CALC_STL) {
			uint16_t z;
			float v1_x, v1_y, v1_z;
			float v2_x, v2_y, v2_z;
			float v3_x, v3_y, v3_z;
			float n_x, n_y, n_z;
			uint16_t temp;
			uint8_t message[24] = { 0x00 };

			for (z = 1; z < Z - 1; z++) {
				for (i = (z * N_MEASUREMENTS) - N_MEASUREMENTS; i < (z * N_MEASUREMENTS) - 1; i++) {
					// Calculate vectors for first "rectangle"
					v1_x = points[i + N_MEASUREMENTS].x - points[i].x;
					v1_y = points[i + N_MEASUREMENTS].y - points[i].y;
					v1_z = points[i + N_MEASUREMENTS].z - points[i].z;

					v2_x = points[i + 1 + N_MEASUREMENTS].x - points[i].x;
					v2_y = points[i + 1 + N_MEASUREMENTS].y - points[i].y;
					v2_z = points[i + 1 + N_MEASUREMENTS].z - points[i].z;

					v3_x = points[i + 1].x - points[i].x;
					v3_y = points[i + 1].y - points[i].y;
					v3_z = points[i + 1].z - points[i].z;

					// calculate normal vector for first triangle of "rectangle"
					n_x = (v1_y * v2_z) - (v1_z * v2_y);
					n_y = (v1_z * v2_x) - (v1_x * v2_z);
					n_z = (v1_x * v2_y) - (v1_y * v2_x);

					// transmit first rectangle
					temp = convert_float_to_int16(&n_x);
					memcpy(message, &temp, sizeof(uint16_t));
					temp = convert_float_to_int16(&n_y);
					memcpy(message + 2, &temp, sizeof(uint16_t));
					temp = convert_float_to_int16(&n_z);
					memcpy(message + 4, &temp, sizeof(uint16_t));
					temp = convert_float_to_int16(&(points[i].x));
					memcpy(message + 6, &temp, sizeof(uint16_t));
					temp = convert_float_to_int16(&(points[i].y));
					memcpy(message + 8, &temp, sizeof(uint16_t));
					temp = convert_float_to_int16(&(points[i].z));
					memcpy(message + 10, &temp, sizeof(uint16_t));
					temp = convert_float_to_int16(&(points[i + N_MEASUREMENTS].x));
					memcpy(message + 12, &temp, sizeof(uint16_t));
					temp = convert_float_to_int16(&(points[i + N_MEASUREMENTS].y));
					memcpy(message + 14, &temp, sizeof(uint16_t));
					temp = convert_float_to_int16(&(points[i + N_MEASUREMENTS].z));
					memcpy(message + 16, &temp, sizeof(uint16_t));
					temp = convert_float_to_int16(&(points[i + 1 + N_MEASUREMENTS].x));
					memcpy(message + 18, &temp, sizeof(uint16_t));
					temp = convert_float_to_int16(&(points[i + 1 + N_MEASUREMENTS].y));
					memcpy(message + 20, &temp, sizeof(uint16_t));
					temp = convert_float_to_int16(&(points[i + 1 + N_MEASUREMENTS].z));
					memcpy(message + 22, &temp, sizeof(uint16_t));

					HAL_UART_Transmit(&huart2, message, sizeof(message), HAL_MAX_DELAY);
					HAL_Delay(10);

					// calculate normal vector for second triangle of "rectangle"
					n_x = (v3_y * v2_z) - (v3_z * v2_y);
					n_y = (v3_z * v2_x) - (v3_x * v2_z);
					n_z = (v3_x * v2_y) - (v3_y * v2_x);

					// transmit second triangle
					temp = convert_float_to_int16(&n_x);
					memcpy(message, &temp, sizeof(uint16_t));
					temp = convert_float_to_int16(&n_y);
					memcpy(message + 2, &temp, sizeof(uint16_t));
					temp = convert_float_to_int16(&n_z);
					memcpy(message + 4, &temp, sizeof(uint16_t));
					temp = convert_float_to_int16(&(points[i].x));
					memcpy(message + 6, &temp, sizeof(uint16_t));
					temp = convert_float_to_int16(&(points[i].y));
					memcpy(message + 8, &temp, sizeof(uint16_t));
					temp = convert_float_to_int16(&(points[i].z));
					memcpy(message + 10, &temp, sizeof(uint16_t));
					temp = convert_float_to_int16(&(points[i + 1 + N_MEASUREMENTS].x));
					memcpy(message + 12, &temp, sizeof(uint16_t));
					temp = convert_float_to_int16(&(points[i + 1 + N_MEASUREMENTS].y));
					memcpy(message + 14, &temp, sizeof(uint16_t));
					temp = convert_float_to_int16(&(points[i + 1 + N_MEASUREMENTS].z));
					memcpy(message + 16, &temp, sizeof(uint16_t));
					temp = convert_float_to_int16(&(points[i + 1].x));
					memcpy(message + 18, &temp, sizeof(uint16_t));
					temp = convert_float_to_int16(&(points[i + 1].y));
					memcpy(message + 20, &temp, sizeof(uint16_t));
					temp = convert_float_to_int16(&(points[i + 1].z));
					memcpy(message + 22, &temp, sizeof(uint16_t));

					HAL_UART_Transmit(&huart2, message, sizeof(message), HAL_MAX_DELAY);
					HAL_Delay(10);

				}

				//
				// Last vertex of row
				//

				v1_x = points[(z * N_MEASUREMENTS) - 1].x - points[(z * N_MEASUREMENTS) + N_MEASUREMENTS - 1].x;
				v1_y = points[(z * N_MEASUREMENTS) - 1].y - points[(z * N_MEASUREMENTS) + N_MEASUREMENTS - 1].y;
				v1_z = points[(z * N_MEASUREMENTS) - 1].z - points[(z * N_MEASUREMENTS) + N_MEASUREMENTS - 1].z;

				v2_x = points[(z * N_MEASUREMENTS) - 1].x - points[(z * N_MEASUREMENTS)].x;
				v2_y = points[(z * N_MEASUREMENTS) - 1].y - points[(z * N_MEASUREMENTS)].y;
				v2_z = points[(z * N_MEASUREMENTS) - 1].z - points[(z * N_MEASUREMENTS)].z;

				v3_x = points[(z * N_MEASUREMENTS) - N_MEASUREMENTS].x - points[(z * N_MEASUREMENTS) - 1].x;
				v3_y = points[(z * N_MEASUREMENTS) - N_MEASUREMENTS].y - points[(z * N_MEASUREMENTS) - 1].y;
				v3_z = points[(z * N_MEASUREMENTS) - N_MEASUREMENTS].z - points[(z * N_MEASUREMENTS) - 1].z;

				n_x = (v1_y * v2_z) - (v1_z * v2_y);
				n_y = (v1_z * v2_x) - (v1_x * v2_z);
				n_z = (v1_x * v2_y) - (v1_y * v2_x);

				temp = convert_float_to_int16(&n_x);
				memcpy(message, &temp, sizeof(uint16_t));
				temp = convert_float_to_int16(&n_y);
				memcpy(message + 2, &temp, sizeof(uint16_t));
				temp = convert_float_to_int16(&n_z);
				memcpy(message + 4, &temp, sizeof(uint16_t));
				temp = convert_float_to_int16(&(points[(z * N_MEASUREMENTS) - 1].x));
				memcpy(message + 6, &temp, sizeof(uint16_t));
				temp = convert_float_to_int16(&(points[(z * N_MEASUREMENTS) - 1].y));
				memcpy(message + 8, &temp, sizeof(uint16_t));
				temp = convert_float_to_int16(&(points[(z * N_MEASUREMENTS) - 1].z));
				memcpy(message + 10, &temp, sizeof(uint16_t));
				temp = convert_float_to_int16(&(points[(z * N_MEASUREMENTS) + N_MEASUREMENTS - 1].x));
				memcpy(message + 12, &temp, sizeof(uint16_t));
				temp = convert_float_to_int16(&(points[(z * N_MEASUREMENTS) + N_MEASUREMENTS - 1].y));
				memcpy(message + 14, &temp, sizeof(uint16_t));
				temp = convert_float_to_int16(&(points[(z * N_MEASUREMENTS) + N_MEASUREMENTS - 1].z));
				memcpy(message + 16, &temp, sizeof(uint16_t));
				temp = convert_float_to_int16(&(points[(z * N_MEASUREMENTS)].x));
				memcpy(message + 18, &temp, sizeof(uint16_t));
				temp = convert_float_to_int16(&(points[(z * N_MEASUREMENTS)].y));
				memcpy(message + 20, &temp, sizeof(uint16_t));
				temp = convert_float_to_int16(&(points[(z * N_MEASUREMENTS)].z));
				memcpy(message + 22, &temp, sizeof(uint16_t));

				HAL_UART_Transmit(&huart2, message, sizeof(message), HAL_MAX_DELAY);
				HAL_Delay(10);

				n_x = (v3_y * v2_z) - (v3_z * v2_y);
				n_y = (v3_z * v2_x) - (v3_x * v2_z);
				n_z = (v3_x * v2_y) - (v3_y * v2_x);

				temp = convert_float_to_int16(&n_x);
				memcpy(message, &temp, sizeof(uint16_t));
				temp = convert_float_to_int16(&n_y);
				memcpy(message + 2, &temp, sizeof(uint16_t));
				temp = convert_float_to_int16(&n_z);
				memcpy(message + 4, &temp, sizeof(uint16_t));
				temp = convert_float_to_int16(&(points[i].x));
				memcpy(message + 6, &temp, sizeof(uint16_t));
				temp = convert_float_to_int16(&(points[i].y));
				memcpy(message + 8, &temp, sizeof(uint16_t));
				temp = convert_float_to_int16(&(points[i].z));
				memcpy(message + 10, &temp, sizeof(uint16_t));
				temp = convert_float_to_int16(&(points[(z * N_MEASUREMENTS)].x));
				memcpy(message + 12, &temp, sizeof(uint16_t));
				temp = convert_float_to_int16(&(points[(z * N_MEASUREMENTS)].y));
				memcpy(message + 14, &temp, sizeof(uint16_t));
				temp = convert_float_to_int16(&(points[(z * N_MEASUREMENTS)].z));
				memcpy(message + 16, &temp, sizeof(uint16_t));
				temp = convert_float_to_int16(&(points[(z * N_MEASUREMENTS) - N_MEASUREMENTS].x));
				memcpy(message + 18, &temp, sizeof(uint16_t));
				temp = convert_float_to_int16(&(points[(z * N_MEASUREMENTS) - N_MEASUREMENTS].y));
				memcpy(message + 20, &temp, sizeof(uint16_t));
				temp = convert_float_to_int16(&(points[(z * N_MEASUREMENTS) - N_MEASUREMENTS].z));
				memcpy(message + 22, &temp, sizeof(uint16_t));

				HAL_UART_Transmit(&huart2, message, sizeof(message), HAL_MAX_DELAY);
				HAL_Delay(10);
			}
			scanner.state = STATE_HOME_LIFT;
		}

		/*--------------------------------------------
		 * STATE: STATE MOVE LIFT
		 * --------------------------------------------*/
		if (scanner.state == STATE_MOVE_LIFT) {

			// Initiate stepper motor movement.
			if (tim2_activated == 0x00) {
				reset_motion_profile(&scanner.lift_profile, 10000.0);
				scanner.lift_profile.enabled = 0x01;
				tim2_activated = 0x01;
				HAL_GPIO_WritePin(LIFT_EN_GPIO_Port, LIFT_EN_Pin, GPIO_PIN_RESET);
				HAL_TIM_Base_Start_IT(&htim2);
			}

			// Stepper motor movement done.
			if (scanner.lift_profile.enabled == 0x00) {
				tim2_activated = 0x00;
				HAL_GPIO_WritePin(LIFT_EN_GPIO_Port, LIFT_EN_Pin, GPIO_PIN_SET);
				HAL_TIM_Base_Stop_IT(&htim2);
				scanner.state = STATE_MOVE_TABLE;
			}
		}

		/*--------------------------------------------
		 * STATE: STATE SCAN ROW
		 * --------------------------------------------*/
		if (scanner.state == STATE_SCAN_ROW) {

			if (scanner.table_profile.pos - measurements[measurement_index - 1].position >= 3.6) {
				measurements[measurement_index].distance = scanner.sensor_a.distance;
				measurements[measurement_index].position = scanner.table_profile.pos;
				measurement_index++;
			}

			if (scanner.table_profile.enabled == 0x00) {
				Z += 1;
				scanner.state = STATE_MOVE_TABLE;
			}
		}

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 80;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 4000-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_DOWN;
  htim2.Init.Period = 1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 8000-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 2500-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 10000-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 80-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 921600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, TABLE_EN_Pin|LIFT_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|LIFT_STEP_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LIFT_DIRECTION_Pin|TABLE_DIRECTION_Pin|TABLE_STEP_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(HIGH_SIGNAL_GPIO_Port, HIGH_SIGNAL_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : TABLE_EN_Pin LIFT_EN_Pin */
  GPIO_InitStruct.Pin = TABLE_EN_Pin|LIFT_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LIFT_DIRECTION_Pin TABLE_DIRECTION_Pin */
  GPIO_InitStruct.Pin = LIFT_DIRECTION_Pin|TABLE_DIRECTION_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : HIGH_SIGNAL_Pin */
  GPIO_InitStruct.Pin = HIGH_SIGNAL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(HIGH_SIGNAL_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LIFT_STEP_Pin */
  GPIO_InitStruct.Pin = LIFT_STEP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(LIFT_STEP_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LIFT_ENDPOS_Pin */
  GPIO_InitStruct.Pin = LIFT_ENDPOS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(LIFT_ENDPOS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : TABLE_STEP_Pin */
  GPIO_InitStruct.Pin = TABLE_STEP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(TABLE_STEP_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/*
 * @brief Calculate 3D-Point from measured distance.
 *
 * @param p Pointer where the calculated point should be saved.
 *
 * @param measurement_index Index where the measured distance get found.
 *
 * @return 0x00 := NO_ERROR
 */
uint8_t calculate_point(tPoint *p, size_t measurement_index)
{
	float r		= 65.0 - (float)measurements[measurement_index].distance;
	float angle = measurements[measurement_index].position / 360.0 * 2.0 * 3.1415; // DEG TO RAD

	p->x = (r * cosf(angle)) + 100.0;
	p->y = (r * sinf(angle)) + 100.0;
	p->z = scanner.lift_profile.pos;

	return 0x00;

}

/*
 * @brief Convert float to 16 bit integer.
 *
 * @param value Pointer to the float value
 *
 * @return converted value
 */
int16_t convert_float_to_int16(float *value)
{
	if(*value > 65535.0 || *value < -65535.0)
		return -1;

	return (int16_t)(*value * 100.0);
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
	  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
