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

#include <math.h>
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

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

#define MONITOR 1 // 1: 6 axis, 2: PID, 3: ESC, 4: Remote control, 5: PID tuning

// Motor PWM values
#define INIT_SPEED 100
#define MIN_SPEED 1400
#define MAX_SPEED 2400

#define MIN_THROTTLE -199
#define MIN_YAW -199
#define MIN_PITCH -199
#define MIN_ROLL -199

#define MIN_PITCH_PROPORTION -(MAX_SPEED - MIN_SPEED)*0.1
#define MAX_PITCH_PROPORTION (MAX_SPEED - MIN_SPEED)*0.1
#define MIN_PITCH_INTEGRAL -(MAX_SPEED - MIN_SPEED)*0.05
#define MAX_PITCH_INTEGRAL (MAX_SPEED - MIN_SPEED)*0.05
#define MIN_PITCH_DERIVATION -(MAX_SPEED - MIN_SPEED)*0.1
#define MAX_PITCH_DERIVATION (MAX_SPEED - MIN_SPEED)*0.1

#define MIN_ROLL_PROPORTION -(MAX_SPEED - MIN_SPEED)*0.1
#define MAX_ROLL_PROPORTION (MAX_SPEED - MIN_SPEED)*0.1
#define MIN_ROLL_INTEGRAL -(MAX_SPEED - MIN_SPEED)*0.05
#define MAX_ROLL_INTEGRAL (MAX_SPEED - MIN_SPEED)*0.05
#define MIN_ROLL_DERIVATION -(MAX_SPEED - MIN_SPEED)*0.1
#define MAX_ROLL_DERIVATION (MAX_SPEED - MIN_SPEED)*0.1

#define MIN_YAW_PROPORTION -(MAX_SPEED - MIN_SPEED)*0.01
#define MAX_YAW_PROPORTION (MAX_SPEED - MIN_SPEED)*0.01
#define MIN_YAW_INTEGRAL -(MAX_SPEED - MIN_SPEED)*0.01
#define MAX_YAW_INTEGRAL (MAX_SPEED - MIN_SPEED)*0.01
#define MIN_YAW_DERIVATION -(MAX_SPEED - MIN_SPEED)*0.01
#define MAX_YAW_DERIVATION (MAX_SPEED - MIN_SPEED)*0.01

// PID
#define P_PITCH_GAIN 1.2
#define I_PITCH_GAIN 0.0
#define I_PITCH_PERIOD 0.0
#define D_PITCH_GAIN 0.4

#define P_ROLL_GAIN 1.2
#define I_ROLL_GAIN 0.0
#define I_ROLL_PERIOD 0.0
#define D_ROLL_GAIN 0.4

#define P_YAW_GAIN 0.0
#define I_YAW_GAIN 0.0 // No use due to drifting P
#define I_YAW_PERIOD 0.0 // No use due to drifting P
#define D_YAW_GAIN 0.0

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

// Fly mode
FlyMode fly_mode = init;

// PID
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
int32_t pwm_in[30];
float g_throttle = 0;
float g_pitch = 0;
float g_roll = 0;
float g_yaw = 0;
float g_tune1 = 0;
float g_tune2 = 0;
int g_stick1 = 0; // 1, 2
int g_stick2 = 0; // 1, 2, 3
int g_stick3 = 0; // 1, 2, 3
int g_stick4 = 0; // 1, 2

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

// PID tuning remote control
int8_t g_pid_tuning = 2; // 1: 1, 2: 0, 3: -1

// Monitor
float monitor[9] = {0};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

void set_speed(uint32_t m1, uint32_t m2,
    uint32_t m3, uint32_t m4) {
  TIM2->CCR1 = m1;
  TIM2->CCR2 = m2;
  TIM2->CCR3 = m3;
  TIM2->CCR4 = m4;
}

void fly(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim7;
extern UART_HandleTypeDef huart1;
/* USER CODE BEGIN EV */

extern average_filter_t g_af[10];

extern mpu6050_t g_mpu6050;
extern ms5611_t g_ms5611;

// Remote control
extern uint8_t g_uart_rx_buffer[10];

extern float limit(float number, float min, float max);
extern void console(const char *str);
extern void send_data(
  float x1, float x2, float x3,
  float x4, float x5, float x6,
  float x7, float x8, float x9);

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
  * @brief This function handles TIM3 global interrupt.
  */
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */

  /* USER CODE END TIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim3);
  /* USER CODE BEGIN TIM3_IRQn 1 */

  /* USER CODE END TIM3_IRQn 1 */
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

  fly();

  /* USER CODE END TIM7_IRQn 0 */
  HAL_TIM_IRQHandler(&htim7);
  /* USER CODE BEGIN TIM7_IRQn 1 */

  /* USER CODE END TIM7_IRQn 1 */
}

/* USER CODE BEGIN 1 */

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_1);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  // To know whether this timer is hanging
  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_1);

  // Update monitor
  send_data(monitor[0], monitor[1], monitor[2],
      monitor[3], monitor[4], monitor[5],
      monitor[6], monitor[7], monitor[8]);
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
  if (htim->Instance == TIM3) {
    switch (htim->Channel) {
      case HAL_TIM_ACTIVE_CHANNEL_1:
        if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6) == GPIO_PIN_SET) {
          pwm_in[0] = HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_1);
        }

        if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6) == GPIO_PIN_RESET) {
          pwm_in[1] = HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_1);
          int value = pwm_in[1] - pwm_in[0];
          if (value >= 400 && value <= 798) { // [400, 798]
            pwm_in[2] = value;
          }
        }
        break;
      case HAL_TIM_ACTIVE_CHANNEL_2:
        if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7) == GPIO_PIN_SET) {
          pwm_in[3] = HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_2);
        }

        if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7) == GPIO_PIN_RESET) {
          pwm_in[4] = HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_2);
          int value = pwm_in[4] - pwm_in[3];
          if (value >= 400 && value <= 798) {
            pwm_in[5] = value;
          }
        }
        break;
      case HAL_TIM_ACTIVE_CHANNEL_3:
        if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) == GPIO_PIN_SET) {
          pwm_in[6] = HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_3);
        }

        if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) == GPIO_PIN_RESET) {
          pwm_in[7] = HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_3);
          int value = pwm_in[7] - pwm_in[6];
          if (value >= 400 && value <= 798) {
            pwm_in[8] = value;
          }
        }
        break;
      case HAL_TIM_ACTIVE_CHANNEL_4:
        if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1) == GPIO_PIN_SET) {
          pwm_in[9] = HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_4);
        }

        if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1) == GPIO_PIN_RESET) {
          pwm_in[10] = HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_4);
          int value = pwm_in[10] - pwm_in[9];
          if (value >= 400 && value <= 798) {
            pwm_in[11] = value;
          }
        }
        break;
      default:
        break;
    }

    g_throttle = average_filter_update(&g_af[0], pwm_in[5] - 599);
    g_yaw = average_filter_update(&g_af[1], pwm_in[2] - 599);
    g_pitch = average_filter_update(&g_af[2], pwm_in[8] - 599);
    g_roll = average_filter_update(&g_af[3], pwm_in[11] - 599);
  }

  if (htim->Instance == TIM4) {
    switch (htim->Channel) {
      case HAL_TIM_ACTIVE_CHANNEL_1:
        if (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_12) == GPIO_PIN_SET) {
          pwm_in[18] = HAL_TIM_ReadCapturedValue(&htim4, TIM_CHANNEL_1);
        }

        if (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_12) == GPIO_PIN_RESET) {
          pwm_in[19] = HAL_TIM_ReadCapturedValue(&htim4, TIM_CHANNEL_1);
          int value = pwm_in[19] - pwm_in[18];
          if (value >= 399 && value <= 799) { // [399, 799]
            pwm_in[20] = value > 600 ? 2 : 1;
          }
        }
        break;
      case HAL_TIM_ACTIVE_CHANNEL_2:
        if (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_13) == GPIO_PIN_SET) {
          pwm_in[21] = HAL_TIM_ReadCapturedValue(&htim4, TIM_CHANNEL_2);
        }

        if (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_13) == GPIO_PIN_RESET) {
          pwm_in[22] = HAL_TIM_ReadCapturedValue(&htim4, TIM_CHANNEL_2);
          int value = pwm_in[22] - pwm_in[21];
          if (value >= 399 && value <= 799) { // [399, 799]
            pwm_in[23] = value > 700 ? 3 : (value < 500 ? 1 : 2);
          }
        }
        break;
      case HAL_TIM_ACTIVE_CHANNEL_3:
        if (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_14) == GPIO_PIN_SET) {
          pwm_in[24] = HAL_TIM_ReadCapturedValue(&htim4, TIM_CHANNEL_3);
        }

        if (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_14) == GPIO_PIN_RESET) {
          pwm_in[25] = HAL_TIM_ReadCapturedValue(&htim4, TIM_CHANNEL_3);
          int value = pwm_in[25] - pwm_in[24];
          if (value >= 399 && value <= 799) {
            pwm_in[26] = value > 700 ? 3 : (value < 500 ? 1 : 2);
          }
        }
        break;
      case HAL_TIM_ACTIVE_CHANNEL_4:
        if (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_15) == GPIO_PIN_SET) {
          pwm_in[27] = HAL_TIM_ReadCapturedValue(&htim4, TIM_CHANNEL_4);
        }

        if (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_15) == GPIO_PIN_RESET) {
          pwm_in[28] = HAL_TIM_ReadCapturedValue(&htim4, TIM_CHANNEL_4);
          int value = pwm_in[28] - pwm_in[27];
          if (value >= 399 && value <= 799) {
            pwm_in[29] = value > 600 ? 2 : 1;
          }
        }
        break;
      default:
        break;
    }

    g_stick1 = pwm_in[20];
    g_stick2 = pwm_in[23];
    g_stick3 = pwm_in[26];
    g_stick4 = pwm_in[29];

    if (g_stick4 == 2) { // Tuning PID
      float add = 0.0;
      if (g_stick3 != g_pid_tuning) {
        add = g_stick3 - 2; // -1, 0, 1
        g_pid_tuning = g_stick3;
      }

      switch (g_stick2) {
        case 1: // Tuning P
          g_P_pitch_gain = limit(g_P_pitch_gain + add / 10, 0, 5);
          g_P_roll_gain = limit(g_P_roll_gain + add / 10, 0, 5);
//          g_P_yaw_gain = limit(g_P_yaw_gain + add / 100, 0, 0.5);
          break;
        case 2: // Tuning I
          g_I_pitch_period = limit(g_I_pitch_period + add / 100, 0, 0.5);
          g_I_roll_period = limit(g_I_roll_period + add / 100, 0, 0.5);
//          g_I_yaw_period = limit(g_I_yaw_period + add / 100, 0, 0.5);
          break;
        case 3: // Tuning D
          g_D_pitch_gain = limit(g_D_pitch_gain + add / 100, 0, 0.5);
          g_D_roll_gain = limit(g_D_roll_gain + add / 100, 0, 0.5);
//          g_D_yaw_gain = limit(g_D_yaw_gain + add / 1000, 0, 0.05);
          break;
        default:
          break;
      }
    }
  }

  if (htim->Instance == TIM5) {
    switch (htim->Channel) {
      case HAL_TIM_ACTIVE_CHANNEL_3:
        if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2) == GPIO_PIN_SET) {
          pwm_in[12] = HAL_TIM_ReadCapturedValue(&htim5, TIM_CHANNEL_3);
        }

        if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2) == GPIO_PIN_RESET) {
          pwm_in[13] = HAL_TIM_ReadCapturedValue(&htim5, TIM_CHANNEL_3);
          int value = pwm_in[13] - pwm_in[12];
          if (value >= 399 && value <= 799) { // [399, 799]
            pwm_in[14] = value;
          }
        }
        break;
      case HAL_TIM_ACTIVE_CHANNEL_4:
        if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3) == GPIO_PIN_SET) {
          pwm_in[15] = HAL_TIM_ReadCapturedValue(&htim5, TIM_CHANNEL_4);
        }

        if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3) == GPIO_PIN_RESET) {
          pwm_in[16] = HAL_TIM_ReadCapturedValue(&htim5, TIM_CHANNEL_4);
          int value = pwm_in[16] - pwm_in[15];
          if (value >= 399 && value <= 799) {
            pwm_in[17] = value;
          }
        }
        break;
      default:
        break;
    }

    g_tune1 = average_filter_update(&g_af[4], pwm_in[14] - 599);
    g_tune2 = average_filter_update(&g_af[5], pwm_in[17] - 599);
  }

#if MONITOR == 4
  monitor[0] = g_throttle;
  monitor[1] = g_yaw;
  monitor[2] = g_tune1;
  monitor[3] = g_pitch;
  monitor[4] = g_roll;
  monitor[5] = g_tune2;
  monitor[6] = g_stick1+g_stick4;
  monitor[7] = g_stick2;
  monitor[8] = g_stick3;
#endif

#if MONITOR == 5
  monitor[0] = g_P_pitch_gain;
  monitor[1] = g_P_roll_gain;
  monitor[2] = g_P_yaw_gain;
  monitor[3] = g_I_pitch_period;
  monitor[4] = g_I_roll_period;
  monitor[5] = g_I_yaw_period;
  monitor[6] = g_D_pitch_gain;
  monitor[7] = g_D_roll_gain;
  monitor[8] = g_D_yaw_gain;
#endif
}

void fly() {
  // Update from sensors
  MPU6050_update(&g_mpu6050);
  MS5611_update(&g_ms5611);

  float angle_x = g_mpu6050.angle_x;
  float angle_y = g_mpu6050.angle_y;
  float angle_z = g_mpu6050.angle_z;
  float gyro_x = g_mpu6050.gx;
  float gyro_y = g_mpu6050.gy;
  float gyro_z = g_mpu6050.gz;

  // Add remote control bias
  angle_y -= 0.5*g_pitch;
  angle_x -= 0.5*g_roll;
  angle_z -= 0.0*g_yaw;

  // Keep alive for the fly
  if (g_stick1 != 2) {
    fly_mode = init;
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

      set_speed(INIT_SPEED, INIT_SPEED, INIT_SPEED, INIT_SPEED);

      // Move sticks to make it ready to take off
      if (g_throttle <= MIN_THROTTLE && g_yaw <= MIN_YAW
          && g_pitch <= MIN_PITCH && g_roll >= MIN_ROLL) {
        fly_mode = ready;
      }

      break;
    case ready:
      // Reset accumulated integral
      g_I_pitch_accumulated = 0;
      g_I_roll_accumulated = 0;
      g_I_yaw_accumulated = 0;

      set_speed(MIN_SPEED, MIN_SPEED, MIN_SPEED, MIN_SPEED);

      // Switch to fly mode
      if (g_throttle > 0) {
        fly_mode = moving;
      }

      break;
    case holding:

      break;
    case moving:
      g_P_pitch = limit(angle_y*g_P_pitch_gain, MIN_PITCH_PROPORTION, MAX_PITCH_PROPORTION);
      g_I_pitch_accumulated += angle_y*I_PITCH_PERIOD; // 0.005 = 1/FREQ
      g_I_pitch_accumulated = limit(g_I_pitch_accumulated, MIN_PITCH_INTEGRAL/g_I_pitch_gain, MAX_PITCH_INTEGRAL/g_I_pitch_gain);
      g_I_pitch = g_I_pitch_accumulated*g_I_pitch_gain;
      g_D_pitch = limit(gyro_x*g_D_pitch_gain, MIN_PITCH_DERIVATION, MAX_PITCH_DERIVATION);

      g_P_roll = limit(angle_x*g_P_roll_gain, MIN_ROLL_PROPORTION, MAX_ROLL_PROPORTION);
      g_I_roll_accumulated += angle_x*I_ROLL_PERIOD;
      g_I_roll_accumulated = limit(g_I_roll_accumulated, MIN_ROLL_INTEGRAL/g_I_roll_gain, MAX_ROLL_INTEGRAL/g_I_roll_gain);
      g_I_roll = g_I_roll_accumulated*g_I_roll_gain;
      g_D_roll = limit(gyro_y*g_D_roll_gain, MIN_ROLL_DERIVATION, MAX_ROLL_DERIVATION);

      g_P_yaw = limit(angle_z*g_P_yaw_gain, MIN_YAW_PROPORTION, MAX_YAW_PROPORTION);
      g_I_yaw_accumulated += angle_z*I_YAW_PERIOD;
      g_I_yaw_accumulated = limit(g_I_yaw_accumulated, MIN_YAW_INTEGRAL/g_I_yaw_gain, MAX_YAW_INTEGRAL/g_I_yaw_gain);
      g_I_yaw = g_I_yaw_accumulated*g_I_yaw_gain;
      g_D_yaw = limit(gyro_z*g_D_yaw_gain, MIN_YAW_DERIVATION, MAX_YAW_DERIVATION);

      int throttle = MIN_SPEED + (int)(10.0f*sqrt(g_throttle));

      g_sig1 = throttle + (g_P_pitch + g_I_pitch + g_D_pitch) - (g_P_roll + g_I_roll + g_D_roll) + (g_P_yaw + g_I_yaw + g_D_yaw);
      g_sig2 = throttle + (g_P_pitch + g_I_pitch + g_D_pitch) + (g_P_roll + g_I_roll + g_D_roll) - (g_P_yaw + g_I_yaw + g_D_yaw);
      g_sig3 = throttle - (g_P_pitch + g_I_pitch + g_D_pitch) + (g_P_roll + g_I_roll + g_D_roll) + (g_P_yaw + g_I_yaw + g_D_yaw);
      g_sig4 = throttle - (g_P_pitch + g_I_pitch + g_D_pitch) - (g_P_roll + g_I_roll + g_D_roll) - (g_P_yaw + g_I_yaw + g_D_yaw);

      g_sig1 = limit(g_sig1, MIN_SPEED, MAX_SPEED);
      g_sig2 = limit(g_sig2, MIN_SPEED, MAX_SPEED);
      g_sig3 = limit(g_sig3, MIN_SPEED, MAX_SPEED);
      g_sig4 = limit(g_sig4, MIN_SPEED, MAX_SPEED);

      set_speed(g_sig1, g_sig2, g_sig3, g_sig4);

      // Pull down the stick to stop
      if (g_throttle <= MIN_THROTTLE) {
        fly_mode = init;
      }

      // Stop if angle too large (crashed), can disable if test with the rig
//      if (angle_x < -70 || angle_x > 70 || angle_y < -70 || angle_y > 70) {
//        fly_mode = init;
//      }

      break;
    case landing:

      break;
    case testing:
      g_sig1 = MIN_SPEED + 6*g_throttle;
      g_sig2 = MIN_SPEED + 6*g_throttle;
      g_sig3 = MIN_SPEED + 6*g_throttle;
      g_sig4 = MIN_SPEED + 6*g_throttle;
      set_speed(g_sig1, g_sig2, g_sig3, g_sig4);
      break;
  }

#if MONITOR == 1
  monitor[0] = angle_x;
  monitor[1] = angle_y;
  monitor[2] = angle_z;
  monitor[3] = gyro_x;
  monitor[4] = gyro_y;
  monitor[5] = gyro_z;
  monitor[6] = g_ms5611.altitude;
  monitor[7] = g_ms5611.altitude;
  monitor[8] = g_ms5611.altitude;
#endif

#if MONITOR == 2
  monitor[0] = g_P_pitch;
  monitor[1] = g_I_pitch;
  monitor[2] = g_D_pitch;
  monitor[3] = g_P_roll;
  monitor[4] = g_I_roll;
  monitor[5] = g_D_roll;
  monitor[6] = g_P_yaw;
  monitor[7] = g_I_yaw;
  monitor[8] = g_D_yaw;
#endif

#if MONITOR == 3
  monitor[0] = g_sig1;
  monitor[1] = g_sig2;
  monitor[2] = g_sig1 > g_sig2 ? g_sig2 : g_sig1;
  monitor[3] = g_sig3;
  monitor[4] = g_sig4;
  monitor[5] = g_sig3 > g_sig4 ? g_sig4 : g_sig3;
#endif
}

/* USER CODE END 1 */
