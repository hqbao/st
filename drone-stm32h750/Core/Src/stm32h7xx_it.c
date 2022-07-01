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
  int lf; // Left forward
  int rf; // Right forward
  int lb; // Left backward
  int rb; // Right backward
  int v1; // Vertical
  int v2; // Vertical
  int v3; // Vertical
  int v4; // Vertical
} drift_t;

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

#define MONITOR 6 // 1: 6 axis, 2: PID, 3: ESC, 4: Remote control, 5: PID tuning, 6: Calibration

// Motor PWM values
#define INIT_SPEED 2000
#define MIN_SPEED 2400
#define MAX_SPEED 4800

#define MIN_THROTTLE 0
#define MIN_YAW -199
#define MIN_PITCH -199
#define MAX_ROLL 199

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

#define MIN_YAW_PROPORTION -(MAX_SPEED - MIN_SPEED)*0.2
#define MAX_YAW_PROPORTION (MAX_SPEED - MIN_SPEED)*0.2
#define MIN_YAW_INTEGRAL -(MAX_SPEED - MIN_SPEED)*0.1
#define MAX_YAW_INTEGRAL (MAX_SPEED - MIN_SPEED)*0.1
#define MIN_YAW_DERIVATION -(MAX_SPEED - MIN_SPEED)*0.2
#define MAX_YAW_DERIVATION (MAX_SPEED - MIN_SPEED)*0.2

// PID
#define P_PITCH_GAIN 10.0 // 10.0
#define I_PITCH_GAIN 0.0 // 0.01
#define I_PITCH_PERIOD 0.0 // 2.0
#define D_PITCH_GAIN 4.0 // 9.0

#define P_ROLL_GAIN 10.0
#define I_ROLL_GAIN 0.0
#define I_ROLL_PERIOD 0.0
#define D_ROLL_GAIN 9.0

#define P_YAW_GAIN 7.0
#define I_YAW_GAIN 0.0 // No use due to drifting P
#define I_YAW_PERIOD 0.0 // No use due to drifting P
#define D_YAW_GAIN 5.0

#define LIMIT(number, min, max) (number < min ? min : (number > max ? max : number))

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

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

// Drift detection
drift_t drift;

// Monitor
char monitor[120];

average_filter_t g_af[5];

#define UART_BUF_SIZE 256
uint8_t g_uart_rx_buffer1[UART_BUF_SIZE];
uint8_t g_uart_rx_buffer2[UART_BUF_SIZE];
uint8_t g_uart_rx_buffer3[UART_BUF_SIZE];
uint8_t g_uart_rx_buffer4[UART_BUF_SIZE];

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

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim7;
extern DMA_HandleTypeDef hdma_uart4_rx;
extern DMA_HandleTypeDef hdma_uart5_rx;
extern DMA_HandleTypeDef hdma_uart7_rx;
extern DMA_HandleTypeDef hdma_uart8_rx;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart7;
extern UART_HandleTypeDef huart8;
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
  * @brief This function handles DMA1 stream0 global interrupt.
  */
void DMA1_Stream0_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream0_IRQn 0 */

  /* USER CODE END DMA1_Stream0_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_uart4_rx);
  /* USER CODE BEGIN DMA1_Stream0_IRQn 1 */

  /* USER CODE END DMA1_Stream0_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream1 global interrupt.
  */
void DMA1_Stream1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream1_IRQn 0 */

  /* USER CODE END DMA1_Stream1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_uart5_rx);
  /* USER CODE BEGIN DMA1_Stream1_IRQn 1 */

  /* USER CODE END DMA1_Stream1_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream2 global interrupt.
  */
void DMA1_Stream2_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream2_IRQn 0 */

  /* USER CODE END DMA1_Stream2_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_uart7_rx);
  /* USER CODE BEGIN DMA1_Stream2_IRQn 1 */

  /* USER CODE END DMA1_Stream2_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream3 global interrupt.
  */
void DMA1_Stream3_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream3_IRQn 0 */

  /* USER CODE END DMA1_Stream3_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_uart8_rx);
  /* USER CODE BEGIN DMA1_Stream3_IRQn 1 */

  /* USER CODE END DMA1_Stream3_IRQn 1 */
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
  * @brief This function handles UART4 global interrupt.
  */
void UART4_IRQHandler(void)
{
  /* USER CODE BEGIN UART4_IRQn 0 */

  /* USER CODE END UART4_IRQn 0 */
  HAL_UART_IRQHandler(&huart4);
  /* USER CODE BEGIN UART4_IRQn 1 */

  /* USER CODE END UART4_IRQn 1 */
}

/**
  * @brief This function handles UART5 global interrupt.
  */
void UART5_IRQHandler(void)
{
  /* USER CODE BEGIN UART5_IRQn 0 */

  /* USER CODE END UART5_IRQn 0 */
  HAL_UART_IRQHandler(&huart5);
  /* USER CODE BEGIN UART5_IRQn 1 */

  /* USER CODE END UART5_IRQn 1 */
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
  * @brief This function handles UART7 global interrupt.
  */
void UART7_IRQHandler(void)
{
  /* USER CODE BEGIN UART7_IRQn 0 */

  /* USER CODE END UART7_IRQn 0 */
  HAL_UART_IRQHandler(&huart7);
  /* USER CODE BEGIN UART7_IRQn 1 */

  /* USER CODE END UART7_IRQn 1 */
}

/**
  * @brief This function handles UART8 global interrupt.
  */
void UART8_IRQHandler(void)
{
  /* USER CODE BEGIN UART8_IRQn 0 */

  /* USER CODE END UART8_IRQn 0 */
  HAL_UART_IRQHandler(&huart8);
  /* USER CODE BEGIN UART8_IRQn 1 */

  /* USER CODE END UART8_IRQn 1 */
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
          if (value >= 350 && value <= 850) { // [400, 798]
            pwm_in[2] = value;
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
          if (value >= 350 && value <= 850) {
            pwm_in[5] = value;
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
          if (value >= 350 && value <= 850) {
            pwm_in[8] = value;
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
          if (value >= 350 && value <= 850) {
            pwm_in[11] = value;
          }
        }
        break;
      default:
        break;
    }

    g_throttle = average_filter_update(&g_af[0], pwm_in[2] - 400);
    g_yaw = average_filter_update(&g_af[1], pwm_in[11] - 600);
    g_pitch = average_filter_update(&g_af[2], pwm_in[8] - 600);
    g_roll = average_filter_update(&g_af[3], pwm_in[5] - 600);
  }

  if (htim->Instance == TIM5) {
    switch (htim->Channel) {
      case HAL_TIM_ACTIVE_CHANNEL_1:
        if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET) {
          pwm_in[12] = HAL_TIM_ReadCapturedValue(&htim5, TIM_CHANNEL_1);
        }

        if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_RESET) {
          pwm_in[13] = HAL_TIM_ReadCapturedValue(&htim5, TIM_CHANNEL_3);
          int value = pwm_in[13] - pwm_in[12];
          if (value >= 350 && value <= 850) { // [399, 799]
            pwm_in[14] = value;
          }
        }
        break;
      default:
        break;
    }

    g_stick1 = average_filter_update(&g_af[4], pwm_in[14] - 400);
  }
}

void schedule_400hz(void) {
  // Update from sensors
//  MPU6050_update(&g_mpu6050);
//  MS5611_update(&g_ms5611);
//  fly();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_1);
}

void schedule_20hz(void) {
  static char msg[16];
  static int starts[4] = {-1, -1, -1, -1};

  for (int t = 0; t < 4; t += 1) {
    int start = starts[t];
    int end = -1;
    uint8_t *p;
    switch (t) {
    case 0: p = g_uart_rx_buffer1; break;
    case 1: p = g_uart_rx_buffer2; break;
    case 2: p = g_uart_rx_buffer3; break;
    case 3: p = g_uart_rx_buffer4; break;
    }

    if (start == -1) {
      for (int i = 0; i < UART_BUF_SIZE; i += 1) {
        if (p[i] == '$') {
          start = i;
          break;
        }
      }
    }

    if (start > -1) {
      for (int i = 0; i < UART_BUF_SIZE; i += 1) {
        if (p[(start+i)%UART_BUF_SIZE] == 0) {
          start = -1;
          break;
        }

        if (p[(start+i)%UART_BUF_SIZE] == '\n') {
          end = i;
          break;
        }
      }
    }

    if (start > -1 && end > -1) {
      memset(msg, 0, 16);

      if (start < end) {
        memcpy(msg, &p[start], end - start);
        memset(&p[start], 0, end - start);
      }
      else if (start > end) {
        memcpy(msg, &p[start], UART_BUF_SIZE - start);
        memset(&p[start], 0, UART_BUF_SIZE - start);
        memcpy(&msg[UART_BUF_SIZE - start], p, end);
        memset(p, 0, end);
      }

      starts[t] = -1;

      int idx = 0;
      for (int idx = 0; idx < 16; idx += 1) {
        if (msg[idx] == ',') {
          break;
        }
      }
      msg[idx] = 0;
      int dy = atoi(&msg[1]);
      int dx = atoi(&msg[idx+1]);
      switch (t) {
      case 0: drift.lf = dx;drift.v1 = dy; break;
      case 1: drift.rf = dx;drift.v2 = dy; break;
      case 2: drift.rb = dx;drift.v3 = dy; break;
      case 3: drift.lb = dx;drift.v4 = dy; break;
      }
    }
  }

  memset(monitor, 0, 64);
  sprintf(monitor, "%d,%d,%d,%d,%d\n",
      drift.lf, drift.rf, drift.lb, drift.rb,
      drift.v1 + drift.v2 + drift.v3 + drift.v4);
  console(monitor);
}

void fly() {
  float angle_x = g_mpu6050.angle_x;
  float angle_y = g_mpu6050.angle_y;
  float angle_z = g_mpu6050.angle_z;
  float gyro_x = g_mpu6050.gyro_x;
  float gyro_y = g_mpu6050.gyro_y;
  float gyro_z = g_mpu6050.gyro_z;

  // Add remote control bias
  float angle_error_y = angle_y - 0.125*g_pitch; // Max 25 degree
  float angle_error_x = angle_x - 0.125*g_roll; // Max 25 degree
  float angle_error_z = angle_z;
  if (g_yaw < -5 || g_yaw > 5) {
    angle_error_z = g_yaw > 0 ? -0.2*g_yaw : -0.2*g_yaw;
    g_mpu6050.angle_z = 0;
  }

  // Keep alive for the fly
  if (g_stick1 < 100) {
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
          && g_pitch <= MIN_PITCH && g_roll >= MAX_ROLL) {
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
      g_P_pitch = LIMIT(angle_error_y*g_P_pitch_gain, MIN_PITCH_PROPORTION, MAX_PITCH_PROPORTION);
      g_I_pitch_accumulated += angle_error_y*I_PITCH_PERIOD; // 0.005 = 1/FREQ
      g_I_pitch_accumulated = LIMIT(g_I_pitch_accumulated, MIN_PITCH_INTEGRAL/g_I_pitch_gain, MAX_PITCH_INTEGRAL/g_I_pitch_gain);
      g_I_pitch = g_I_pitch_accumulated*g_I_pitch_gain;
      g_D_pitch = LIMIT(gyro_x*g_D_pitch_gain, MIN_PITCH_DERIVATION, MAX_PITCH_DERIVATION);

      g_P_roll = LIMIT(angle_error_x*g_P_roll_gain, MIN_ROLL_PROPORTION, MAX_ROLL_PROPORTION);
      g_I_roll_accumulated += angle_error_x*I_ROLL_PERIOD;
      g_I_roll_accumulated = LIMIT(g_I_roll_accumulated, MIN_ROLL_INTEGRAL/g_I_roll_gain, MAX_ROLL_INTEGRAL/g_I_roll_gain);
      g_I_roll = g_I_roll_accumulated*g_I_roll_gain;
      g_D_roll = LIMIT(gyro_y*g_D_roll_gain, MIN_ROLL_DERIVATION, MAX_ROLL_DERIVATION);

      g_P_yaw = LIMIT(angle_error_z*g_P_yaw_gain, MIN_YAW_PROPORTION, MAX_YAW_PROPORTION);
      g_I_yaw_accumulated += angle_error_z*I_YAW_PERIOD;
      g_I_yaw_accumulated = LIMIT(g_I_yaw_accumulated, MIN_YAW_INTEGRAL/g_I_yaw_gain, MAX_YAW_INTEGRAL/g_I_yaw_gain);
      g_I_yaw = g_I_yaw_accumulated*g_I_yaw_gain;
      g_D_yaw = LIMIT(gyro_z*g_D_yaw_gain, MIN_YAW_DERIVATION, MAX_YAW_DERIVATION);

      int throttle = MIN_SPEED + (int)(70.0f*sqrt(g_throttle));

      g_sig1 = throttle + (g_P_pitch + g_I_pitch + g_D_pitch) - (g_P_roll + g_I_roll + g_D_roll) - (g_P_yaw + g_I_yaw + g_D_yaw);
      g_sig2 = throttle + (g_P_pitch + g_I_pitch + g_D_pitch) + (g_P_roll + g_I_roll + g_D_roll) + (g_P_yaw + g_I_yaw + g_D_yaw);
      g_sig3 = throttle - (g_P_pitch + g_I_pitch + g_D_pitch) + (g_P_roll + g_I_roll + g_D_roll) - (g_P_yaw + g_I_yaw + g_D_yaw);
      g_sig4 = throttle - (g_P_pitch + g_I_pitch + g_D_pitch) - (g_P_roll + g_I_roll + g_D_roll) + (g_P_yaw + g_I_yaw + g_D_yaw);

      g_sig1 = LIMIT(g_sig1, MIN_SPEED, MAX_SPEED);
      g_sig2 = LIMIT(g_sig2, MIN_SPEED, MAX_SPEED);
      g_sig3 = LIMIT(g_sig3, MIN_SPEED, MAX_SPEED);
      g_sig4 = LIMIT(g_sig4, MIN_SPEED, MAX_SPEED);

      set_speed(g_sig1, g_sig2, g_sig3, g_sig4);

      // Pull down the stick to stop
      if (g_throttle <= MIN_THROTTLE) {
        fly_mode = init;
      }

      // Stop if angle too large (crashed), can disable if test with the rig
      if (angle_error_x < -90 || angle_error_x > 90 || angle_error_y < -90 || angle_error_y > 90) {
        fly_mode = init;
      }

      break;
    case landing:

      break;
    case testing:
      g_sig1 = MIN_SPEED - 100 + LIMIT(5*g_throttle, 0, 4800);
      g_sig2 = MIN_SPEED - 100 + LIMIT(5*g_yaw, 0, 4800);
      g_sig3 = MIN_SPEED - 100 + LIMIT(5*g_pitch, 0, 4800);
      g_sig4 = MIN_SPEED - 100 + LIMIT(5*g_roll, 0, 4800);

      // Pull down the stick to stop
      if (g_throttle <= MIN_THROTTLE) {
        fly_mode = init;
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
  average_filter_init(&g_af[0], 5); // Thrust
  average_filter_init(&g_af[1], 5); // Yaw
  average_filter_init(&g_af[2], 5); // Pitch
  average_filter_init(&g_af[3], 5); // Roll
  average_filter_init(&g_af[4], 5); // Stick 1
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

  int i = 0;
  int gx = 0;
  int gy = 0;
  int gz = 0;
  while (i < 1000) {
    MPU6050_update(&g_mpu6050);
    HAL_Delay(3);
    gx += g_mpu6050.gx;
    gy += g_mpu6050.gy;
    gz += g_mpu6050.gz;
    i += 1;
  }
  gx = gx/1000;
  gy = gy/1000;
  gz = gz/1000;

  MPU6050_set_offset(&g_mpu6050, 0, 0, 0, -gx, -gy, -gz);

  // This takes quite long
  while (1) {
    int error = MS5611_init(
        &g_ms5611,
        &hi2c1);
    if (error == 0) break;
    flash(error);
  }
}

/* USER CODE END 1 */
