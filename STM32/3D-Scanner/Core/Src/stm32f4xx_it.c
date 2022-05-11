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
 * @file stm32f4xx_it.c
 * @copyright Copyright 2022 Jean-Marcel Herzog. This project is released under the MIT license.
 * @date 11.05.2022
 * @version 1
*/
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "machine_parameter.h"
#include "motion_profile.h"
#include "vl6180x.h"
#include "types.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */
extern tScanner scanner;
/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
void oneStepTable();
void oneStepLift();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief Stepper Motor Timer TIM2 Interrupt.
  */
void TIM2_IRQHandler(void) {
	/* USER CODE BEGIN TIM2_IRQn 0 */

	/* ----------------------------- */
	/* Table Movement Update Section */
	/* ----------------------------- */
	if (scanner.table_profile.enabled == 0x01) {
		scanner.table_profile.v = scanner.table_profile.v + scanner.table_profile.A;
		if (scanner.table_profile.v < scanner.table_profile.v_sf)
			scanner.table_profile.v = scanner.table_profile.v_sf;
		scanner.table_profile.p = scanner.table_profile.p
				+ scanner.table_profile.v;
		scanner.table_profile.pos = scanner.table_profile.p / 12800 * 360.0;
		if (scanner.table_profile.p - scanner.table_profile.np >= 1) {
			oneStepTable();
			scanner.table_profile.np++;
		}

		if (scanner.table_profile.np >= scanner.table_profile.N_a
				&& scanner.table_profile.np
						< scanner.table_profile.N - scanner.table_profile.N_d)
			scanner.table_profile.A = 0;
		else if (scanner.table_profile.np
				>= scanner.table_profile.N - scanner.table_profile.N_d)
			scanner.table_profile.A = scanner.table_profile.D;
		if (scanner.table_profile.np >= scanner.table_profile.N) {
			scanner.table_profile.enabled = 0x00;
			HAL_TIM_Base_Stop_IT(&htim2);
		}
	}

	/* ---------------------------------------------- */
	/* Linear-Axis Movement for move to home position */
	/* (Fast move down until end-switch reached)      */
	/* ---------------------------------------------- */

	if (scanner.state == STATE_HOME_LIFT
			&& scanner.lift_profile.enabled == 0x01) {
		oneStepLift();
	}
	/* ------------------------------------------------ */
	/* Linear-Axis Movement for motion profile movement */
	/* ------------------------------------------------ */
	else if (scanner.lift_profile.enabled == 0x01) {
		scanner.lift_profile.v = scanner.lift_profile.v + scanner.lift_profile.A;
		if (scanner.lift_profile.v < scanner.lift_profile.v_sf)
			scanner.lift_profile.v = scanner.lift_profile.v_sf;
		scanner.lift_profile.p = scanner.lift_profile.p + scanner.lift_profile.v;

		if (scanner.lift_dir == 0x01)
			scanner.lift_profile.pos += ((float) scanner.lift_profile.p / 1024.0) - scanner.lift_profile.dp;
		else
			scanner.lift_profile.pos -= ((float) scanner.lift_profile.p / 1024.0) - scanner.lift_profile.dp;

		scanner.lift_profile.dp = ((float) scanner.lift_profile.p / 1024.0);

		if (scanner.lift_profile.p - scanner.lift_profile.np >= 1) {
			oneStepLift();
			scanner.lift_profile.np++;
		}

		if (scanner.lift_profile.np >= scanner.lift_profile.N_a && scanner.lift_profile.np < scanner.lift_profile.N - scanner.lift_profile.N_d)
			scanner.lift_profile.A = 0;
		else if (scanner.lift_profile.np >= scanner.lift_profile.N - scanner.lift_profile.N_d)
			scanner.lift_profile.A = scanner.lift_profile.D;
		if (scanner.lift_profile.np >= scanner.lift_profile.N) {
			scanner.lift_profile.enabled = 0x00;
			HAL_TIM_Base_Stop_IT(&htim2);

		}
	}

	/* USER CODE END TIM2_IRQn 0 */
	HAL_TIM_IRQHandler(&htim2);
	/* USER CODE BEGIN TIM2_IRQn 1 */

	/* USER CODE END TIM2_IRQn 1 */
}

/**
  * @brief Evaluation Board LED TIM3 Interrupt.
  */
void TIM3_IRQHandler(void)
{
	/* USER CODE BEGIN TIM3_IRQn 0 */
	HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
	/* USER CODE END TIM3_IRQn 0 */
	HAL_TIM_IRQHandler(&htim3);
	/* USER CODE BEGIN TIM3_IRQn 1 */

	/* USER CODE END TIM3_IRQn 1 */
}

/**
  * @brief Distance sensor measurement interrupt.
  */
void TIM4_IRQHandler(void)
{
	/* USER CODE BEGIN TIM4_IRQn 0 */
#ifdef SIM_MODE
	//scanner.sensor_a.distance = 10.0 + (scanner.lift_profile.pos * 0.25);
	scanner.sensor_a.distance = 25.0;
#else
	if (scanner.sensor_a.trig == 0x00)
	{
		DistanceSensor_Trigger(&scanner.sensor_a);
	}
	else
	{
		DistanceSensor_TrigResponse(&scanner.sensor_a);
		DistanceSensor_Trigger(&scanner.sensor_a);
	}
#endif
	/* USER CODE END TIM4_IRQn 0 */
	HAL_TIM_IRQHandler(&htim4);
	/* USER CODE BEGIN TIM4_IRQn 1 */

	/* USER CODE END TIM4_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/**
 * @brief Perform one step table stepper movement.
 */
void oneStepTable()
{
	HAL_GPIO_TogglePin(TABLE_STEP_GPIO_Port, TABLE_STEP_Pin);
	for(size_t i = 0; i < 10; i++) {}	// short delay (stepper need min 2.5 us high pulse)
	HAL_GPIO_TogglePin(TABLE_STEP_GPIO_Port, TABLE_STEP_Pin);
}

/**
 * @brief Perform one step linear-axis stepper movement.
 */
void oneStepLift()
{
	HAL_GPIO_TogglePin(LIFT_STEP_GPIO_Port, LIFT_STEP_Pin);
	for (size_t i = 0; i < 10; i++) {} // short delay (stepper need min 2.5 us high pulse)
	HAL_GPIO_TogglePin(LIFT_STEP_GPIO_Port, LIFT_STEP_Pin);
}

/* USER CODE END 1 */
