/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32h7xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32h7xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include "kalman.h"
#include "gy86.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

typedef enum {
  init = 0,
  ready,
  holding,
  moving,
  landing,
  testing
} FlyMode;

typedef struct {
  // External drift detection
  // Drift detection by optical flow cameras
  int v_front;
  int front;
  int v_back;
  int back;
  int v_left;
  int left;
  int v_right;
  int right;
  int bottom_y;
  int bottom_x;
  int dy;
  int dx;
  int dh;
} drift_t;

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

#define MONITOR 2 // 1: Calibration,
                  // 2: 6 axis,
                  // 3: ESC,
                  // 4: Remote control,
                  // 5: PID,
                  // 6: Drift
                  // 7: Height
                  // 8: PID Error

// Motor PWM values
#define INIT_SPEED 2400
#define MIN_SPEED (INIT_SPEED + 200)
#define MAX_SPEED 5200

#define MIN_PWN_IN_CAP 249
#define MAX_PWN_IN_CAP 498
#define RANGE_PWM_IN_CAP (MAX_PWN_IN_CAP - MIN_PWN_IN_CAP)

#define OPTICALFLOW_AVG_PWM 24

#define MIN_THROTTLE 0
#define MAX_THROTTLE RANGE_PWM_IN_CAP
#define MIN_YAW ((int)(-RANGE_PWM_IN_CAP/2)+2)
#define MIN_PITCH ((int)(-RANGE_PWM_IN_CAP/2)+2)
#define MAX_ROLL ((int)(RANGE_PWM_IN_CAP/2)-2)

#define MIN_PITCH_PROPORTION -(MAX_SPEED - MIN_SPEED)*0.3
#define MAX_PITCH_PROPORTION (MAX_SPEED - MIN_SPEED)*0.3
#define MIN_PITCH_INTEGRAL -(MAX_SPEED - MIN_SPEED)*0.1
#define MAX_PITCH_INTEGRAL (MAX_SPEED - MIN_SPEED)*0.1
#define MIN_PITCH_DERIVATION -(MAX_SPEED - MIN_SPEED)*0.3
#define MAX_PITCH_DERIVATION (MAX_SPEED - MIN_SPEED)*0.3

#define MIN_ROLL_PROPORTION -(MAX_SPEED - MIN_SPEED)*0.3
#define MAX_ROLL_PROPORTION (MAX_SPEED - MIN_SPEED)*0.3
#define MIN_ROLL_INTEGRAL -(MAX_SPEED - MIN_SPEED)*0.05
#define MAX_ROLL_INTEGRAL (MAX_SPEED - MIN_SPEED)*0.05
#define MIN_ROLL_DERIVATION -(MAX_SPEED - MIN_SPEED)*0.3
#define MAX_ROLL_DERIVATION (MAX_SPEED - MIN_SPEED)*0.3

#define MIN_YAW_PROPORTION -(MAX_SPEED - MIN_SPEED)*0.3
#define MAX_YAW_PROPORTION (MAX_SPEED - MIN_SPEED)*0.3
#define MIN_YAW_INTEGRAL -(MAX_SPEED - MIN_SPEED)*0.1
#define MAX_YAW_INTEGRAL (MAX_SPEED - MIN_SPEED)*0.1
#define MIN_YAW_DERIVATION -(MAX_SPEED - MIN_SPEED)*0.3
#define MAX_YAW_DERIVATION (MAX_SPEED - MIN_SPEED)*0.3

// PID
#define P_PITCH_GAIN 7.0 // 10.0
#define I_PITCH_GAIN 0.0 // 0.01
#define I_PITCH_PERIOD 0.0 // 2.0
#define D_PITCH_GAIN 1.6 // 9.0

#define P_ROLL_GAIN 7.0
#define I_ROLL_GAIN 0.0
#define I_ROLL_PERIOD 0.0
#define D_ROLL_GAIN 1.6

#define P_YAW_GAIN 7.0
#define I_YAW_GAIN 0.0 // No use due to drifting P
#define I_YAW_PERIOD 0.0 // No use due to drifting P
#define D_YAW_GAIN 2.0

#define DRIFT_GAIN 0.0

#define LIMIT(number, min, max) (number < min ? min : (number > max ? max : number))

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

// PID
float g_angle_error_x = 0;
float g_angle_error_y = 0;
float g_angle_error_z = 0;

float g_P_pitch = 0;
float g_I_pitch = 0;
float g_I_pitch_accumulated = 0;
float g_D_pitch = 0;

float g_P_roll = 0;
float g_I_roll = 0;
float g_I_roll_accumulated = 0;
float g_D_roll = 0;

float g_P_yaw = 0;
float g_I_yaw = 0;
float g_I_yaw_accumulated = 0;
float g_D_yaw = 0;

float g_sig1 = 0;
float g_sig2 = 0;
float g_sig3 = 0;
float g_sig4 = 0;

// Remote control
int32_t pwm_in[64];
float g_throttle = 0;
float g_pitch = 0;
float g_roll = 0;
float g_yaw = 0;
float g_stick1 = 0; // 1, 2

// Remote control
float g_P_pitch_gain = P_PITCH_GAIN;
float g_I_pitch_gain = I_PITCH_GAIN;
float g_I_pitch_period = I_PITCH_PERIOD;
float g_D_pitch_gain = D_PITCH_GAIN;
float g_P_roll_gain = P_ROLL_GAIN;
float g_I_roll_gain = I_ROLL_GAIN;
float g_I_roll_period = I_ROLL_PERIOD;
float g_D_roll_gain = D_ROLL_GAIN;
float g_P_yaw_gain = P_YAW_GAIN;
float g_I_yaw_gain = I_YAW_GAIN;
float g_I_yaw_period = I_YAW_PERIOD;
float g_D_yaw_gain = D_YAW_GAIN;

float g_height = 0;

// Drift detection
drift_t drift = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

// Monitor
char monitor[120];

average_filter_t g_af[5];
kalman_filter_t g_kf[7];

// Gyroscope
mpu6050_t g_mpu6050;

// Pressure
ms5611_t g_ms5611;

// Fly mode
FlyMode fly_mode = init;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

void console(const char *str);
void set_speed(uint32_t m1, uint32_t m2, uint32_t m3, uint32_t m4);
void schedule_400hz(void);
void schedule_20hz(void);
void fly(void);

void blink(void) {
  static int blink = 0;
  if (blink == 40) HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_1);
  if (blink >= 40) blink = 0;
  blink += 1;
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_i2c1_rx;
extern I2C_HandleTypeDef hi2c1;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim7;
extern TIM_HandleTypeDef htim8;
extern TIM_HandleTypeDef htim17;
extern UART_HandleTypeDef huart1;
/* USER CODE BEGIN EV */

extern I2C_HandleTypeDef hi2c1;

extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart7;
extern UART_HandleTypeDef huart8;

extern void flash(uint8_t count);

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex Processor Interruption and Exception Handlers          */
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
/* STM32H7xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32h7xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 stream4 global interrupt.
  */
void DMA1_Stream4_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream4_IRQn 0 */

  /* USER CODE END DMA1_Stream4_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_i2c1_rx);
  /* USER CODE BEGIN DMA1_Stream4_IRQn 1 */

  /* USER CODE END DMA1_Stream4_IRQn 1 */
}

/**
  * @brief This function handles TIM4 global interrupt.
  */
void TIM4_IRQHandler(void)
{
  /* USER CODE BEGIN TIM4_IRQn 0 */

  /* USER CODE END TIM4_IRQn 0 */
  HAL_TIM_IRQHandler(&htim4);
  /* USER CODE BEGIN TIM4_IRQn 1 */

  /* USER CODE END TIM4_IRQn 1 */
}

/**
  * @brief This function handles I2C1 event interrupt.
  */
void I2C1_EV_IRQHandler(void)
{
  /* USER CODE BEGIN I2C1_EV_IRQn 0 */

  /* USER CODE END I2C1_EV_IRQn 0 */
  HAL_I2C_EV_IRQHandler(&hi2c1);
  /* USER CODE BEGIN I2C1_EV_IRQn 1 */

  /* USER CODE END I2C1_EV_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */

  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}

/**
  * @brief This function handles TIM8 capture compare interrupt.
  */
void TIM8_CC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM8_CC_IRQn 0 */

  /* USER CODE END TIM8_CC_IRQn 0 */
  HAL_TIM_IRQHandler(&htim8);
  /* USER CODE BEGIN TIM8_CC_IRQn 1 */

  /* USER CODE END TIM8_CC_IRQn 1 */
}

/**
  * @brief This function handles TIM5 global interrupt.
  */
void TIM5_IRQHandler(void)
{
  /* USER CODE BEGIN TIM5_IRQn 0 */

  /* USER CODE END TIM5_IRQn 0 */
  HAL_TIM_IRQHandler(&htim5);
  /* USER CODE BEGIN TIM5_IRQn 1 */

  /* USER CODE END TIM5_IRQn 1 */
}

/**
  * @brief This function handles TIM6 global interrupt, DAC1_CH1 and DAC1_CH2 underrun error interrupts.
  */
void TIM6_DAC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM6_DAC_IRQn 0 */

  schedule_400hz();

  /* USER CODE END TIM6_DAC_IRQn 0 */
  HAL_TIM_IRQHandler(&htim6);
  /* USER CODE BEGIN TIM6_DAC_IRQn 1 */

  /* USER CODE END TIM6_DAC_IRQn 1 */
}

/**
  * @brief This function handles TIM7 global interrupt.
  */
void TIM7_IRQHandler(void)
{
  /* USER CODE BEGIN TIM7_IRQn 0 */

  schedule_20hz();

  /* USER CODE END TIM7_IRQn 0 */
  HAL_TIM_IRQHandler(&htim7);
  /* USER CODE BEGIN TIM7_IRQn 1 */

  /* USER CODE END TIM7_IRQn 1 */
}

/**
  * @brief This function handles TIM17 global interrupt.
  */
void TIM17_IRQHandler(void)
{
  /* USER CODE BEGIN TIM17_IRQn 0 */

  /* USER CODE END TIM17_IRQn 0 */
  HAL_TIM_IRQHandler(&htim17);
  /* USER CODE BEGIN TIM17_IRQn 1 */

  /* USER CODE END TIM17_IRQn 1 */
}

/* USER CODE BEGIN 1 */

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
  if (htim->Instance == TIM4) {
    switch (htim->Channel) {
      case HAL_TIM_ACTIVE_CHANNEL_1:
        if (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_12) == GPIO_PIN_SET) {
          pwm_in[0] = HAL_TIM_ReadCapturedValue(&htim4, TIM_CHANNEL_1);
        }

        if (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_12) == GPIO_PIN_RESET) {
          pwm_in[1] = HAL_TIM_ReadCapturedValue(&htim4, TIM_CHANNEL_1);
          int value = pwm_in[1] - pwm_in[0];
          if (value >= MIN_PWN_IN_CAP-20 && value <= MAX_PWN_IN_CAP+20) {
            pwm_in[2] = value;
            float throttle = average_filter_update(&g_af[0], pwm_in[2] - MIN_PWN_IN_CAP - RANGE_PWM_IN_CAP/2);
            if (abs(throttle) > 5) g_throttle = LIMIT(g_throttle + 0.01*throttle, MIN_THROTTLE, MAX_THROTTLE);
          }
        }
        break;
      case HAL_TIM_ACTIVE_CHANNEL_2:
        if (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_13) == GPIO_PIN_SET) {
          pwm_in[3] = HAL_TIM_ReadCapturedValue(&htim4, TIM_CHANNEL_2);
        }

        if (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_13) == GPIO_PIN_RESET) {
          pwm_in[4] = HAL_TIM_ReadCapturedValue(&htim4, TIM_CHANNEL_2);
          int value = pwm_in[4] - pwm_in[3];
          if (value >= MIN_PWN_IN_CAP-20 && value <= MAX_PWN_IN_CAP+20) {
            pwm_in[5] = value;
            float yaw = average_filter_update(&g_af[1], pwm_in[5] - MIN_PWN_IN_CAP - RANGE_PWM_IN_CAP/2);
            if (abs(g_yaw - yaw) > 1) g_yaw = yaw;
          }
        }
        break;
      case HAL_TIM_ACTIVE_CHANNEL_3:
        if (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_14) == GPIO_PIN_SET) {
          pwm_in[6] = HAL_TIM_ReadCapturedValue(&htim4, TIM_CHANNEL_3);
        }

        if (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_14) == GPIO_PIN_RESET) {
          pwm_in[7] = HAL_TIM_ReadCapturedValue(&htim4, TIM_CHANNEL_3);
          int value = pwm_in[7] - pwm_in[6];
          if (value >= MIN_PWN_IN_CAP-20 && value <= MAX_PWN_IN_CAP+20) {
            pwm_in[8] = value;
            float roll = average_filter_update(&g_af[3], pwm_in[8] - MIN_PWN_IN_CAP - RANGE_PWM_IN_CAP/2);
            if (abs(g_roll - roll) > 1) g_roll = roll;
          }
        }
        break;
      case HAL_TIM_ACTIVE_CHANNEL_4:
        if (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_15) == GPIO_PIN_SET) {
          pwm_in[9] = HAL_TIM_ReadCapturedValue(&htim4, TIM_CHANNEL_4);
        }

        if (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_15) == GPIO_PIN_RESET) {
          pwm_in[10] = HAL_TIM_ReadCapturedValue(&htim4, TIM_CHANNEL_4);
          int value = pwm_in[10] - pwm_in[9];
          if (value >= MIN_PWN_IN_CAP-20 && value <= MAX_PWN_IN_CAP+20) {
            pwm_in[11] = value;
            float pitch = average_filter_update(&g_af[2], pwm_in[11] - MIN_PWN_IN_CAP - RANGE_PWM_IN_CAP/2);
            if (abs(g_pitch - pitch) > 1) g_pitch = pitch;
          }
        }
        break;
      default:
        break;
    }
  }

  if (htim->Instance == TIM5) {
    switch (htim->Channel) {
      case HAL_TIM_ACTIVE_CHANNEL_1:
        if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET) {
          pwm_in[12] = HAL_TIM_ReadCapturedValue(&htim5, TIM_CHANNEL_1);
        }

        if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_RESET) {
          pwm_in[13] = HAL_TIM_ReadCapturedValue(&htim5, TIM_CHANNEL_1);
          int value = pwm_in[13] - pwm_in[12];
          if (value >= MIN_PWN_IN_CAP-20 && value <= MAX_PWN_IN_CAP+20) {
            pwm_in[14] = value;
            g_stick1 = pwm_in[14] > MIN_PWN_IN_CAP + 0.5*RANGE_PWM_IN_CAP ? 1 : 0;
          }
        }
        break;
      case HAL_TIM_ACTIVE_CHANNEL_3:
        if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2) == GPIO_PIN_SET) {
          pwm_in[15] = HAL_TIM_ReadCapturedValue(&htim5, TIM_CHANNEL_3);
        }

        if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2) == GPIO_PIN_RESET) {
          pwm_in[16] = HAL_TIM_ReadCapturedValue(&htim5, TIM_CHANNEL_3);
          int value = pwm_in[16] - pwm_in[15];
          if (value >= OPTICALFLOW_AVG_PWM - 10 && value <= OPTICALFLOW_AVG_PWM + 10) {
            pwm_in[17] = value;
            drift.bottom_x = kalman_filter_update(&g_kf[1], pwm_in[17] - OPTICALFLOW_AVG_PWM);
            drift.dx = abs(drift.front) < abs(drift.bottom_x) ? drift.front : drift.bottom_x;
          }
        }
        break;
      case HAL_TIM_ACTIVE_CHANNEL_4:
        if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3) == GPIO_PIN_SET) {
          pwm_in[18] = HAL_TIM_ReadCapturedValue(&htim5, TIM_CHANNEL_4);
        }

        if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3) == GPIO_PIN_RESET) {
          pwm_in[19] = HAL_TIM_ReadCapturedValue(&htim5, TIM_CHANNEL_4);
          int value = pwm_in[19] - pwm_in[18];
          if (value >= OPTICALFLOW_AVG_PWM - 10 && value <= OPTICALFLOW_AVG_PWM + 10) {
            pwm_in[20] = value;
            drift.bottom_y = kalman_filter_update(&g_kf[0], pwm_in[20] - OPTICALFLOW_AVG_PWM);
            drift.dy = abs(drift.right) < abs(drift.bottom_y) ? drift.right : drift.bottom_y;
          }
        }
        break;
      default:
        break;
    }
  }

  if (htim->Instance == TIM8) {
    switch (htim->Channel) {
      case HAL_TIM_ACTIVE_CHANNEL_1:
        if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_6) == GPIO_PIN_SET) {
          pwm_in[21] = HAL_TIM_ReadCapturedValue(&htim8, TIM_CHANNEL_1);
        }

        if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_6) == GPIO_PIN_RESET) {
          pwm_in[22] = HAL_TIM_ReadCapturedValue(&htim8, TIM_CHANNEL_1);
          int value = pwm_in[22] - pwm_in[21];
          if (value >= OPTICALFLOW_AVG_PWM - 10 && value <= OPTICALFLOW_AVG_PWM + 10) {
            pwm_in[23] = value;
            drift.front = kalman_filter_update(&g_kf[2], pwm_in[23] - OPTICALFLOW_AVG_PWM);
            drift.dx = abs(drift.front) < abs(drift.bottom_x) ? drift.front : drift.bottom_x;
          }
        }
        break;
      case HAL_TIM_ACTIVE_CHANNEL_2:
        if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_7) == GPIO_PIN_SET) {
          pwm_in[24] = HAL_TIM_ReadCapturedValue(&htim8, TIM_CHANNEL_2);
        }

        if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_7) == GPIO_PIN_RESET) {
          pwm_in[25] = HAL_TIM_ReadCapturedValue(&htim8, TIM_CHANNEL_2);
          int value = pwm_in[25] - pwm_in[24];
          if (value >= OPTICALFLOW_AVG_PWM - 10 && value <= OPTICALFLOW_AVG_PWM + 10) {
            pwm_in[26] = value;
            drift.v_front = kalman_filter_update(&g_kf[3], pwm_in[26] - OPTICALFLOW_AVG_PWM);
            drift.dh = abs(drift.v_front) < abs(drift.v_right) ? drift.v_front : drift.v_right;
          }
        }
        break;
      case HAL_TIM_ACTIVE_CHANNEL_3:
        if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_8) == GPIO_PIN_SET) {
          pwm_in[27] = HAL_TIM_ReadCapturedValue(&htim8, TIM_CHANNEL_3);
        }

        if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_8) == GPIO_PIN_RESET) {
          pwm_in[28] = HAL_TIM_ReadCapturedValue(&htim8, TIM_CHANNEL_3);
          int value = pwm_in[28] - pwm_in[27];
          if (value >= OPTICALFLOW_AVG_PWM - 10 && value <= OPTICALFLOW_AVG_PWM + 10) {
            pwm_in[29] = value;
            drift.right = kalman_filter_update(&g_kf[4], pwm_in[29] - OPTICALFLOW_AVG_PWM);
            drift.dy = abs(drift.right) < abs(drift.bottom_y) ? drift.right : drift.bottom_y;
          }
        }
        break;
      case HAL_TIM_ACTIVE_CHANNEL_4:
        if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_9) == GPIO_PIN_SET) {
          pwm_in[30] = HAL_TIM_ReadCapturedValue(&htim8, TIM_CHANNEL_4);
        }

        if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9) == GPIO_PIN_RESET) {
          pwm_in[31] = HAL_TIM_ReadCapturedValue(&htim8, TIM_CHANNEL_4);
          int value = pwm_in[31] - pwm_in[30];
          if (value >= OPTICALFLOW_AVG_PWM - 10 && value <= OPTICALFLOW_AVG_PWM + 10) {
            pwm_in[32] = value;
            drift.v_right = kalman_filter_update(&g_kf[5], pwm_in[32] - OPTICALFLOW_AVG_PWM);
            drift.dh = abs(drift.v_front) < abs(drift.v_right) ? drift.v_front : drift.v_right;
          }
        }
        break;
      default:
        break;
    }
  }

  static char measuring = 0;
  if (measuring == 0) {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
    measuring = 1;
  }

  if (measuring == 1 && htim->Instance == TIM17) {
    switch (htim->Channel) {
      case HAL_TIM_ACTIVE_CHANNEL_1:
        if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9) == GPIO_PIN_RESET) {
          pwm_in[33] = HAL_TIM_ReadCapturedValue(&htim17, TIM_CHANNEL_1);
        }

        if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9) == GPIO_PIN_SET) {
          pwm_in[34] = HAL_TIM_ReadCapturedValue(&htim17, TIM_CHANNEL_1);
          int value = pwm_in[34] - pwm_in[33];
          pwm_in[35] = value;
          g_height = kalman_filter_update(&g_kf[6], pwm_in[35]);
          measuring = 0;
        }
        break;
      default:
        break;
    }
  }
}

void schedule_400hz(void) {
  // Update from sensors
  MPU6050_update(&g_mpu6050);
//  MS5611_update(&g_ms5611);
  fly();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {

}

void schedule_20hz(void) {
#if MONITOR == 1
  memset(monitor, 0, 64);
  sprintf(monitor, "$%d,%d,%d,%d,%d,%d\n",
      (int)(g_mpu6050.ax_offset*1000), (int)(g_mpu6050.ay_offset*1000), (int)(g_mpu6050.az_offset*1000),
      (int)(g_mpu6050.gx_offset*1000), (int)(g_mpu6050.gy_offset*1000), (int)(g_mpu6050.gz_offset*1000));
  console(monitor);
#endif // Calibration

#if MONITOR == 2
  memset(monitor, 0, 64);
  sprintf(monitor, "$%d,%d,%d,%d,%d,%d\n",
      (int)g_mpu6050.angle_x, (int)g_mpu6050.angle_y, (int)g_mpu6050.angle_z,
      (int)g_mpu6050.gyro_x, (int)g_mpu6050.gyro_y, (int)g_mpu6050.gyro_z);
  console(monitor);
#endif // 6 axis

#if MONITOR == 3
  memset(monitor, 0, 120);
  sprintf(monitor, "$%d,%d,%d,%d\n",
      (int)g_sig1, (int)g_sig2, (int)g_sig3, (int)g_sig4);
  console(monitor);
#endif // ESC

#if MONITOR == 4
  memset(monitor, 0, 120);
  sprintf(monitor, "$%d,%d,0,%d,%d,%d\n",
      (int)g_throttle, (int)g_yaw, (int)g_pitch, (int)g_roll, (int)g_stick1);
  console(monitor);
#endif // Remote control

#if MONITOR == 5
  memset(monitor, 0, 64);
  sprintf(monitor, "$%d,%d,%d,%d,%d,%d\n",
      (int)g_P_pitch, (int)g_I_pitch, (int)g_D_pitch,
      (int)g_P_roll, (int)g_I_roll, (int)g_D_roll);
  console(monitor);
#endif // 6 axis

#if MONITOR == 6
  memset(monitor, 0, 64);
//  sprintf(monitor, "$%d,%d,%d,%d,%d,%d\n",
//      drift.front, drift.v_front, drift.bottom_x,
//      drift.right, drift.v_right, drift.bottom_y);
//  console(monitor);
  sprintf(monitor, "$%d,%d,%d\n",
      drift.dy, drift.dx, drift.dh);
  console(monitor);
#endif // Drift

#if MONITOR == 7
  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_1);
  memset(monitor, 0, 64);
  sprintf(monitor, "$%d,%d,%d,%d\n",
      (int)(g_ms5611.P*10-960000),
      (int)(g_ms5611.slow_pressure*1000-960000),
      (int)(g_ms5611.fast_pressure*1000-960000),
      (int)(g_height));
  console(monitor);
#endif // Pressure

#if MONITOR == 8
  memset(monitor, 0, 64);
  sprintf(monitor, "$%d,%d,%d\n",
      (int)g_angle_error_x, (int)g_angle_error_y, (int)g_angle_error_z);
  console(monitor);
#endif // 6 axis
}

void fly() {
  float angle_x = g_mpu6050.angle_x;
  float angle_y = g_mpu6050.angle_y;
  float angle_z = g_mpu6050.angle_z;
  float gyro_x = g_mpu6050.gyro_x;
  float gyro_y = g_mpu6050.gyro_y;
  float gyro_z = g_mpu6050.gyro_z;
  float g_drift_pitch = 0;
  float g_drift_roll = 0;

  // Add remote control bias
  g_angle_error_y = angle_y - 0.4*g_pitch; // Max 50 degree
  g_angle_error_x = angle_x - 0.4*g_roll; // Max 50 degree
  g_angle_error_z = angle_z;

  // On control with remote
  if (abs(g_yaw) > 5 || abs(g_pitch) < 5 || abs(g_roll) < 5) {
    // Add yaw angle to make it turning
    float imbalance_coef = g_yaw > 0 ? 1.5 : 1.5; // For imbalance central mass drone
    g_angle_error_z = LIMIT(-imbalance_coef*g_yaw, -90, 90);

    // Reset the yaw angle so that it doesn't have to recover after force turning
    g_mpu6050.angle_z = 0;

    // Recover from drift detection
    g_drift_pitch = LIMIT(DRIFT_GAIN*drift.dy, -50, 50);
    g_drift_roll = LIMIT(DRIFT_GAIN*drift.dx, -50, 50);
  }

  // Keep alive for the fly
  static int stop_counter = 0;
  if (g_stick1 == 0) {
    if (stop_counter >= 20) {
      fly_mode = init;
    }

    stop_counter += 1;
  }

  switch (fly_mode) {
    case init:
      g_P_pitch_gain = P_PITCH_GAIN;
      g_I_pitch_gain = I_PITCH_GAIN;
      g_I_pitch_period = I_PITCH_PERIOD;
      g_D_pitch_gain = D_PITCH_GAIN;
      g_P_roll_gain = P_ROLL_GAIN;
      g_I_roll_gain = I_ROLL_GAIN;
      g_I_roll_period = I_ROLL_PERIOD;
      g_D_roll_gain = D_ROLL_GAIN;
      g_P_yaw_gain = P_YAW_GAIN;
      g_I_yaw_gain = I_YAW_GAIN;
      g_I_yaw_period = I_YAW_PERIOD;
      g_D_yaw_gain = D_YAW_GAIN;

      // Reset counter before take off
      stop_counter = 0;

      set_speed(INIT_SPEED, INIT_SPEED, INIT_SPEED, INIT_SPEED);

      // Move sticks to make it ready to take off
      if (g_throttle <= MIN_THROTTLE && g_yaw <= MIN_YAW
          && g_pitch <= MIN_PITCH && g_roll >= MAX_ROLL) {
        fly_mode = ready;
      }

      break;
    case ready:
      // Reset accumulated integral before take off
      g_I_pitch_accumulated = 0;
      g_I_roll_accumulated = 0;
      g_I_yaw_accumulated = 0;

      // Reset angle before take off
      g_mpu6050.angle_z = 0;

      set_speed(MIN_SPEED, MIN_SPEED, MIN_SPEED, MIN_SPEED);

      // Switch to fly mode
      if (g_throttle > 5) {
        fly_mode = moving;
      }

      break;
    case holding:

      break;
    case moving:
      g_P_pitch = LIMIT(g_angle_error_y*g_P_pitch_gain, MIN_PITCH_PROPORTION, MAX_PITCH_PROPORTION);
      g_I_pitch_accumulated += g_angle_error_y*I_PITCH_PERIOD; // 0.005 = 1/FREQ
      g_I_pitch_accumulated = LIMIT(g_I_pitch_accumulated, MIN_PITCH_INTEGRAL/g_I_pitch_gain, MAX_PITCH_INTEGRAL/g_I_pitch_gain);
      g_I_pitch = g_I_pitch_accumulated*g_I_pitch_gain;
      g_D_pitch = LIMIT(gyro_x*g_D_pitch_gain, MIN_PITCH_DERIVATION, MAX_PITCH_DERIVATION);

      g_P_roll = LIMIT(g_angle_error_x*g_P_roll_gain, MIN_ROLL_PROPORTION, MAX_ROLL_PROPORTION);
      g_I_roll_accumulated += g_angle_error_x*I_ROLL_PERIOD;
      g_I_roll_accumulated = LIMIT(g_I_roll_accumulated, MIN_ROLL_INTEGRAL/g_I_roll_gain, MAX_ROLL_INTEGRAL/g_I_roll_gain);
      g_I_roll = g_I_roll_accumulated*g_I_roll_gain;
      g_D_roll = LIMIT(gyro_y*g_D_roll_gain, MIN_ROLL_DERIVATION, MAX_ROLL_DERIVATION);

      g_P_yaw = LIMIT(g_angle_error_z*g_P_yaw_gain, MIN_YAW_PROPORTION, MAX_YAW_PROPORTION);
      g_I_yaw_accumulated += g_angle_error_z*I_YAW_PERIOD;
      g_I_yaw_accumulated = LIMIT(g_I_yaw_accumulated, MIN_YAW_INTEGRAL/g_I_yaw_gain, MAX_YAW_INTEGRAL/g_I_yaw_gain);
      g_I_yaw = g_I_yaw_accumulated*g_I_yaw_gain;
      g_D_yaw = LIMIT(gyro_z*g_D_yaw_gain, MIN_YAW_DERIVATION, MAX_YAW_DERIVATION);

      float background = MIN_SPEED + 5*(15.81f*sqrt(g_throttle));

      g_sig1 = background + (g_P_pitch + g_I_pitch + g_D_pitch + g_drift_pitch) - (g_P_roll + g_I_roll + g_D_roll + g_drift_roll) + (g_P_yaw + g_I_yaw + g_D_yaw);
      g_sig2 = background + (g_P_pitch + g_I_pitch + g_D_pitch + g_drift_pitch) + (g_P_roll + g_I_roll + g_D_roll + g_drift_roll) - (g_P_yaw + g_I_yaw + g_D_yaw);
      g_sig3 = background - (g_P_pitch + g_I_pitch + g_D_pitch + g_drift_pitch) + (g_P_roll + g_I_roll + g_D_roll + g_drift_roll) + (g_P_yaw + g_I_yaw + g_D_yaw);
      g_sig4 = background - (g_P_pitch + g_I_pitch + g_D_pitch + g_drift_pitch) - (g_P_roll + g_I_roll + g_D_roll + g_drift_roll) - (g_P_yaw + g_I_yaw + g_D_yaw);

      g_sig1 = LIMIT(g_sig1, MIN_SPEED, MAX_SPEED);
      g_sig2 = LIMIT(g_sig2, MIN_SPEED, MAX_SPEED);
      g_sig3 = LIMIT(g_sig3, MIN_SPEED, MAX_SPEED);
      g_sig4 = LIMIT(g_sig4, MIN_SPEED, MAX_SPEED);

      set_speed(g_sig1, g_sig2, g_sig3, g_sig4);

      // Pull down the stick to stop
      if (g_throttle <= MIN_THROTTLE) {
        if (stop_counter >= 10) {
          fly_mode = init;
        }

        stop_counter += 1;
      }

      // Stop if angle too large (crashed), can disable if test with the rig
      if (g_angle_error_x < -90 || g_angle_error_x > 90 || g_angle_error_y < -90 || g_angle_error_y > 90) {
        fly_mode = init;
      }

      blink();
      break;
    case landing:

      break;
    case testing:
      blink();
      g_sig1 = MIN_SPEED + LIMIT(10*g_throttle, 0, MAX_SPEED);
      g_sig2 = MIN_SPEED + LIMIT(20*g_yaw, 0, MAX_SPEED);
      g_sig3 = MIN_SPEED + LIMIT(20*g_pitch, 0, MAX_SPEED);
      g_sig4 = MIN_SPEED + LIMIT(20*g_roll, 0, MAX_SPEED);

      // Pull down the stick to stop
      if (g_throttle <= MIN_THROTTLE) {
        if (stop_counter >= 10) {
          fly_mode = init;
        }

        stop_counter += 1;
      }

      set_speed(g_sig1, g_sig2, g_sig3, g_sig4);
      break;
  }
}

void console(const char *str) {
  HAL_UART_Transmit_IT(&huart1, (uint8_t*)str, (uint16_t)strlen(str));
}

void set_speed(uint32_t m1, uint32_t m2, uint32_t m3, uint32_t m4) {
  TIM2->CCR1 = m1;
  TIM2->CCR2 = m2;
  TIM2->CCR3 = m3;
  TIM2->CCR4 = m4;
}

void init_filters() {
  average_filter_init(&g_af[0], 5);
  average_filter_init(&g_af[1], 5);
  average_filter_init(&g_af[2], 5);
  average_filter_init(&g_af[3], 5);
  average_filter_init(&g_af[4], 5);

  kalman_filter_init(&g_kf[0], 2, 2, 0.01);
  kalman_filter_init(&g_kf[1], 2, 2, 0.01);
  kalman_filter_init(&g_kf[2], 2, 2, 0.01);
  kalman_filter_init(&g_kf[3], 2, 2, 0.01);
  kalman_filter_init(&g_kf[4], 2, 2, 0.01);
  kalman_filter_init(&g_kf[5], 2, 2, 0.01);
  kalman_filter_init(&g_kf[6], 2, 2, 0.01);
}

void init_sensors() {
  // Init gy-86
  while (1) {
    int error = MPU6050_init(
        &g_mpu6050,
        &hi2c1,
        MPU6050_DataRate_8KHz,
        MPU6050_Accelerometer_16G,
        MPU6050_Gyroscope_2000s);
    if (error == 0) break;
    flash(error);
  }

  MPU6050_calibrate(&g_mpu6050);

  // This takes quite long
//  while (1) {
//    int error = MS5611_init(
//        &g_ms5611,
//        &hi2c1);
//    if (error == 0) break;
//    flash(error);
//  }
}

/* USER CODE END 1 */
