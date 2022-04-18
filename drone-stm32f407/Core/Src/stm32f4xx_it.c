/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
  testing_1,
  testing_2
} FlyMode;

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define MONITOR 1 // 1: 6 axis, 2: PID, 3: ESC, 4: Remote control

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

// Motor PWM values
#define INIT_SPEED 15 // 10
#define MIN_SPEED 550 // 572
#define MAX_SPEED 1050 // 1071

#define MIN_PITCH_PROPORTION -(MAX_SPEED - MIN_SPEED)*0.1
#define MAX_PITCH_PROPORTION (MAX_SPEED - MIN_SPEED)*0.1
#define MIN_PITCH_INTEGRAL -(MAX_SPEED - MIN_SPEED)*0.04
#define MAX_PITCH_INTEGRAL (MAX_SPEED - MIN_SPEED)*0.04
#define MIN_PITCH_DERIVATION -(MAX_SPEED - MIN_SPEED)*0.1
#define MAX_PITCH_DERIVATION (MAX_SPEED - MIN_SPEED)*0.1

#define MIN_ROLL_PROPORTION -(MAX_SPEED - MIN_SPEED)*0.1
#define MAX_ROLL_PROPORTION (MAX_SPEED - MIN_SPEED)*0.1
#define MIN_ROLL_INTEGRAL -(MAX_SPEED - MIN_SPEED)*0.04
#define MAX_ROLL_INTEGRAL (MAX_SPEED - MIN_SPEED)*0.04
#define MIN_ROLL_DERIVATION -(MAX_SPEED - MIN_SPEED)*0.1
#define MAX_ROLL_DERIVATION (MAX_SPEED - MIN_SPEED)*0.1

#define MIN_YAW_PROPORTION -(MAX_SPEED - MIN_SPEED)*0.02
#define MAX_YAW_PROPORTION (MAX_SPEED - MIN_SPEED)*0.02
#define MIN_YAW_INTEGRAL -(MAX_SPEED - MIN_SPEED)*0.02
#define MAX_YAW_INTEGRAL (MAX_SPEED - MIN_SPEED)*0.02
#define MIN_YAW_DERIVATION -(MAX_SPEED - MIN_SPEED)*0.02
#define MAX_YAW_DERIVATION (MAX_SPEED - MIN_SPEED)*0.02

// PID
#define P_PITCH_GAIN 2.00 // 2.00
#define I_PITCH_GAIN 0.05 // 0.1
#define I_PITCH_PERIOD 0.1 // 0.005 = 1/FREQ
#define D_PITCH_GAIN 0.015 // 0.01

#define P_ROLL_GAIN 2.00 // 2.00
#define I_ROLL_GAIN 0.1 // 0.1
#define I_ROLL_PERIOD 0.1 // 0.005 = 1/FREQ
#define D_ROLL_GAIN 0.015 // 0.01

#define P_YAW_GAIN 1.0 //
#define I_YAW_GAIN 0.05 //
#define I_YAW_PERIOD 0.005 // 0.005 = 1/FREQ
#define D_YAW_GAIN 0.01 // 0.01

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
float g_thrust = 0;
float g_pitch = 0;
float g_roll = 0;
float g_yaw = 0;
float g_tune1 = 0;
float g_tune2 = 0;
uint8_t g_stick1 = 0; // 1, 2
uint8_t g_stick2 = 0; // 1, 2, 3
uint8_t g_stick3 = 0; // 1, 2
uint8_t g_stick4 = 0; // 1, 2, 3

// UART control
float g_P_pitch_gain = P_PITCH_GAIN;
float g_I_pitch_gain = I_PITCH_GAIN;
float g_D_pitch_gain = D_PITCH_GAIN;
float g_P_roll_gain = P_ROLL_GAIN;
float g_I_roll_gain = I_ROLL_GAIN;
float g_D_roll_gain = D_ROLL_GAIN;
float g_P_yaw_gain = P_YAW_GAIN;
float g_I_yaw_gain = I_YAW_GAIN;
float g_D_yaw_gain = D_YAW_GAIN;

// Monitor
float monitor[9] = {0};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

extern float limit(float number, float min, float max);

void ctl_motors_speed(uint32_t m1, uint32_t m2,
    uint32_t m3, uint32_t m4) {
  TIM1->CCR1 = m1;
  TIM1->CCR2 = m2;
  TIM1->CCR3 = m3;
  TIM1->CCR4 = m4;
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern I2C_HandleTypeDef hi2c1;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim9;
extern UART_HandleTypeDef huart1;
/* USER CODE BEGIN EV */

extern kalman_filter_t g_filters[10];

extern mpu6050_t g_mpu6050;
extern ms5611_t g_ms5611;

// Remote control
extern uint8_t g_uart_rx_buffer[10];

extern void console(const char *str);
extern void send_data(
  float x1, float x2, float x3,
  float x4, float x5, float x6,
  float x7, float x8, float x9);

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
  * @brief This function handles TIM1 break interrupt and TIM9 global interrupt.
  */
void TIM1_BRK_TIM9_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_BRK_TIM9_IRQn 0 */

  /* USER CODE END TIM1_BRK_TIM9_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  HAL_TIM_IRQHandler(&htim9);
  /* USER CODE BEGIN TIM1_BRK_TIM9_IRQn 1 */

  /* USER CODE END TIM1_BRK_TIM9_IRQn 1 */
}

/**
  * @brief This function handles TIM3 global interrupt.
  */
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */

  // Activate reading
  MPU6050_update(&g_mpu6050);
  MS5611_update(&g_ms5611);

  // Norm gyro [-1, 1]
  float gyro_x = g_mpu6050.gx;
  float gyro_y = g_mpu6050.gy;
  float gyro_z = g_mpu6050.gz;

  // Add control terms
  float angle_x = g_mpu6050.angle_x - 0.5*g_pitch;
  float angle_y = g_mpu6050.angle_y - 0.5*g_roll;
  float angle_z = g_mpu6050.angle_z - 0.0*g_yaw;

  // Move sticks to make it ready to take off
  if (g_thrust <= -99 && g_yaw <= -99
      && g_pitch <= -99 && g_roll >= 98) {
    fly_mode = ready;
  }

  // Keep alive for the fly
  if (g_stick1 != 2) {
    g_I_pitch_accumulated = 0;
    g_I_roll_accumulated = 0;
    g_I_yaw_accumulated = 0;
    ctl_motors_speed(INIT_SPEED, INIT_SPEED, INIT_SPEED, INIT_SPEED);
    fly_mode = init;
  }

  // Stop if angle too large (crashed)
  if (angle_x < -60 || angle_x > 60 || angle_y < -60 || angle_y > 60) {
    g_I_pitch_accumulated = 0;
    g_I_roll_accumulated = 0;
    g_I_yaw_accumulated = 0;
    ctl_motors_speed(INIT_SPEED, INIT_SPEED, INIT_SPEED, INIT_SPEED);
    fly_mode = init;
  }

  switch (fly_mode) {
    case init:
      g_I_pitch_accumulated = 0;
      g_I_roll_accumulated = 0;
      g_I_yaw_accumulated = 0;

      ctl_motors_speed(INIT_SPEED, INIT_SPEED, INIT_SPEED, INIT_SPEED);
      break;
    case ready:
      g_I_pitch_accumulated = 0;
      g_I_roll_accumulated = 0;
      g_I_yaw_accumulated = 0;

      ctl_motors_speed(MIN_SPEED, MIN_SPEED, MIN_SPEED, MIN_SPEED);
      if (g_thrust > 0) {
        fly_mode = testing_2;
      }

      break;
    case holding:
      ctl_motors_speed(MIN_SPEED, MIN_SPEED, MIN_SPEED, MIN_SPEED);
      break;
    case moving:
      ctl_motors_speed(MIN_SPEED, MIN_SPEED, MIN_SPEED, MIN_SPEED);
      break;
    case landing:
      ctl_motors_speed(MIN_SPEED, MIN_SPEED, MIN_SPEED, MIN_SPEED);
      break;
    case testing_1:
      g_sig1 = MIN_SPEED + g_thrust;
      g_sig2 = MIN_SPEED + g_thrust;
      g_sig3 = MIN_SPEED + g_thrust;
      g_sig4 = MIN_SPEED + g_thrust;
      ctl_motors_speed(g_sig1, g_sig2, g_sig3, g_sig4);

      if (g_thrust <= -99) {
        fly_mode = init;
      }

      break;
    case testing_2:
      g_P_pitch = limit(angle_x*g_P_pitch_gain, MIN_PITCH_PROPORTION, MAX_PITCH_PROPORTION);
      g_I_pitch_accumulated += angle_x*I_PITCH_PERIOD; // 0.005 = 1/FREQ
      g_I_pitch = limit(g_I_pitch_accumulated*g_I_pitch_gain, MIN_PITCH_INTEGRAL, MAX_PITCH_INTEGRAL);
      g_D_pitch = limit(gyro_y*g_D_pitch_gain, MIN_PITCH_DERIVATION, MAX_PITCH_DERIVATION);

      g_P_roll = limit(angle_y*g_P_roll_gain, MIN_ROLL_PROPORTION, MAX_ROLL_PROPORTION);
      g_I_roll_accumulated += angle_y*I_ROLL_PERIOD;
      g_I_roll = limit(g_I_roll_accumulated*g_I_roll_gain, MIN_ROLL_INTEGRAL, MAX_ROLL_INTEGRAL);
      g_D_roll = limit(gyro_x*g_D_roll_gain, MIN_ROLL_DERIVATION, MAX_ROLL_DERIVATION);

      g_P_yaw = limit(angle_z*g_P_yaw_gain, MIN_YAW_PROPORTION, MAX_YAW_PROPORTION);
      g_I_yaw_accumulated += angle_z*I_YAW_PERIOD;
      g_I_yaw = limit(g_I_yaw_accumulated*g_I_yaw_gain, MIN_YAW_INTEGRAL, MAX_YAW_INTEGRAL);
      g_D_yaw = limit(gyro_z*g_D_yaw_gain, MIN_YAW_DERIVATION, MAX_YAW_DERIVATION);

      int thrust = MIN_SPEED + g_thrust*3;

      g_sig1 = thrust + (g_P_pitch + g_I_pitch + g_D_pitch) - (g_P_roll + g_I_roll + g_D_roll) + (g_P_yaw + g_I_yaw + g_D_yaw);
      g_sig2 = thrust + (g_P_pitch + g_I_pitch + g_D_pitch) + (g_P_roll + g_I_roll + g_D_roll) - (g_P_yaw + g_I_yaw + g_D_yaw);
      g_sig3 = thrust - (g_P_pitch + g_I_pitch + g_D_pitch) + (g_P_roll + g_I_roll + g_D_roll) + (g_P_yaw + g_I_yaw + g_D_yaw);
      g_sig4 = thrust - (g_P_pitch + g_I_pitch + g_D_pitch) - (g_P_roll + g_I_roll + g_D_roll) - (g_P_yaw + g_I_yaw + g_D_yaw);

      g_sig1 = limit(g_sig1, MIN_SPEED, MAX_SPEED);
      g_sig2 = limit(g_sig2, MIN_SPEED, MAX_SPEED);
      g_sig3 = limit(g_sig3, MIN_SPEED, MAX_SPEED);
      g_sig4 = limit(g_sig4, MIN_SPEED, MAX_SPEED);

      ctl_motors_speed(g_sig1, g_sig2, g_sig3, g_sig4);

      if (g_thrust <= -99) {
        fly_mode = init;
      }

      break;
  }

#if MONITOR == 1
  monitor[0] = angle_x;
  monitor[1] = angle_y;
  monitor[2] = angle_z;
  monitor[3] = gyro_x;
  monitor[4] = gyro_y;
  monitor[5] = gyro_z;
  monitor[6] = (float)g_ms5611.P/100.f;
  monitor[7] = g_ms5611.fast_pressure;
  monitor[8] = g_ms5611.slow_pressure;
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
  * @brief This function handles I2C1 error interrupt.
  */
void I2C1_ER_IRQHandler(void)
{
  /* USER CODE BEGIN I2C1_ER_IRQn 0 */

  /* USER CODE END I2C1_ER_IRQn 0 */
  HAL_I2C_ER_IRQHandler(&hi2c1);
  /* USER CODE BEGIN I2C1_ER_IRQn 1 */

  /* USER CODE END I2C1_ER_IRQn 1 */
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

/* USER CODE BEGIN 1 */

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
  // To know whether this timer is hanging
  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_7);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  // To know whether this timer is hanging
  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_6);

//  // Serialise control values
//  static uint8_t g_uart_rx_buffer_1st_idx = 0;
//  if (g_uart_rx_buffer[0] == 254) g_uart_rx_buffer_1st_idx = 1;
//  if (g_uart_rx_buffer[1] == 254) g_uart_rx_buffer_1st_idx = 2;
//  if (g_uart_rx_buffer[2] == 254) g_uart_rx_buffer_1st_idx = 3;
//  if (g_uart_rx_buffer[3] == 254) g_uart_rx_buffer_1st_idx = 4;
//  if (g_uart_rx_buffer[4] == 254) g_uart_rx_buffer_1st_idx = 5;
//  if (g_uart_rx_buffer[5] == 254) g_uart_rx_buffer_1st_idx = 6;
//  if (g_uart_rx_buffer[6] == 254) g_uart_rx_buffer_1st_idx = 7;
//  if (g_uart_rx_buffer[7] == 254) g_uart_rx_buffer_1st_idx = 8;
//  if (g_uart_rx_buffer[8] == 254) g_uart_rx_buffer_1st_idx = 9;
//  if (g_uart_rx_buffer[9] == 254) g_uart_rx_buffer_1st_idx = 0;
//  g_P_pitch_gain = g_uart_rx_buffer[g_uart_rx_buffer_1st_idx];
//  g_I_pitch_gain = g_uart_rx_buffer[(g_uart_rx_buffer_1st_idx+1)%10];
//  g_D_pitch_gain = g_uart_rx_buffer[(g_uart_rx_buffer_1st_idx+2)%10];
//  g_P_roll_gain = g_uart_rx_buffer[(g_uart_rx_buffer_1st_idx+3)%10];
//  g_I_roll_gain = g_uart_rx_buffer[(g_uart_rx_buffer_1st_idx+4)%10];
//  g_D_roll_gain = g_uart_rx_buffer[(g_uart_rx_buffer_1st_idx+5)%10];
//  g_P_yaw_gain = g_uart_rx_buffer[(g_uart_rx_buffer_1st_idx+6)%10];
//  g_I_yaw_gain = g_uart_rx_buffer[(g_uart_rx_buffer_1st_idx+7)%10];
//  g_D_yaw_gain = g_uart_rx_buffer[(g_uart_rx_buffer_1st_idx+8)%10];

  // Update monitor
  send_data(monitor[0], monitor[1], monitor[2],
      monitor[3], monitor[4], monitor[5],
      monitor[6], monitor[7], monitor[8]);
}

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
          if (value >= 200 && value < 400) {
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
          if (value >= 200 && value < 400) {
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
          if (value >= 200 && value < 400) {
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
          if (value >= 200 && value < 400) {
            pwm_in[11] = value;
          }
        }
        break;
      default:
        break;
    }

    g_thrust = kalman_filter_update(&g_filters[0], pwm_in[5] - 300);
    g_yaw = kalman_filter_update(&g_filters[1], pwm_in[2] - 300);
    g_pitch = kalman_filter_update(&g_filters[2], pwm_in[8] - 300);
    g_roll = kalman_filter_update(&g_filters[3], pwm_in[11] - 300);
  }

  if (htim->Instance == TIM9) {
    switch (htim->Channel) {
      case HAL_TIM_ACTIVE_CHANNEL_1:
        if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_5) == GPIO_PIN_SET) {
          pwm_in[12] = HAL_TIM_ReadCapturedValue(&htim9, TIM_CHANNEL_1);
        }

        if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_5) == GPIO_PIN_RESET) {
          pwm_in[13] = HAL_TIM_ReadCapturedValue(&htim9, TIM_CHANNEL_1);
          int value = pwm_in[13] - pwm_in[12];
          if (value >= 200 && value < 400) {
            pwm_in[14] = value;
          }
        }
        break;
      case HAL_TIM_ACTIVE_CHANNEL_2:
        if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_6) == GPIO_PIN_SET) {
          pwm_in[15] = HAL_TIM_ReadCapturedValue(&htim9, TIM_CHANNEL_2);
        }

        if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_6) == GPIO_PIN_RESET) {
          pwm_in[16] = HAL_TIM_ReadCapturedValue(&htim9, TIM_CHANNEL_2);
          int value = pwm_in[16] - pwm_in[15];
          if (value >= 200 && value < 400) {
            pwm_in[17] = value;
          }
        }
        break;
      default:
        break;
    }

    g_tune1 = kalman_filter_update(&g_filters[4], pwm_in[17] - 300);
    g_tune2 = kalman_filter_update(&g_filters[5], pwm_in[14] - 300);
  }

  if (htim->Instance == TIM5) {
    switch (htim->Channel) {
      case HAL_TIM_ACTIVE_CHANNEL_1:
        if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET) {
          pwm_in[18] = HAL_TIM_ReadCapturedValue(&htim5, TIM_CHANNEL_1);
        }

        if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_RESET) {
          pwm_in[19] = HAL_TIM_ReadCapturedValue(&htim5, TIM_CHANNEL_1);
          int value = pwm_in[19] - pwm_in[18];
          if (value >= 200 && value < 400) { // valid range [200, 400)
            pwm_in[20] = value > 300 ? 2 : 1;
          }
        }
        break;
      case HAL_TIM_ACTIVE_CHANNEL_2:
        if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1) == GPIO_PIN_SET) {
          pwm_in[21] = HAL_TIM_ReadCapturedValue(&htim5, TIM_CHANNEL_2);
        }

        if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1) == GPIO_PIN_RESET) {
          pwm_in[22] = HAL_TIM_ReadCapturedValue(&htim5, TIM_CHANNEL_2);
          int value = pwm_in[22] - pwm_in[21];
          if (value >= 200 && value < 400) { // valid range [200, 400)
            pwm_in[23] = value < 250 ? 1 : (value > 350 ? 3 : 2);
          }
        }
        break;
      case HAL_TIM_ACTIVE_CHANNEL_3:
        if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2) == GPIO_PIN_SET) {
          pwm_in[24] = HAL_TIM_ReadCapturedValue(&htim5, TIM_CHANNEL_3);
        }

        if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2) == GPIO_PIN_RESET) {
          pwm_in[25] = HAL_TIM_ReadCapturedValue(&htim5, TIM_CHANNEL_3);
          int value = pwm_in[25] - pwm_in[24];
          if (value >= 200 && value < 400) { // valid range [200, 400)
            pwm_in[26] = value < 250 ? 1 : (value > 350 ? 3 : 2);
          }
        }
        break;
      case HAL_TIM_ACTIVE_CHANNEL_4:
        if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3) == GPIO_PIN_SET) {
          pwm_in[27] = HAL_TIM_ReadCapturedValue(&htim5, TIM_CHANNEL_4);
        }

        if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3) == GPIO_PIN_RESET) {
          pwm_in[28] = HAL_TIM_ReadCapturedValue(&htim5, TIM_CHANNEL_4);
          int value = pwm_in[28] - pwm_in[27];
          if (value >= 200 && value < 400) { // valid range [200, 400)
            pwm_in[29] = value > 300 ? 2 : 1;
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
  }

#if MONITOR == 4
  monitor[0] = g_thrust;
  monitor[1] = g_yaw;
  monitor[2] = g_tune1;
  monitor[3] = g_pitch;
  monitor[4] = g_roll;
  monitor[5] = g_tune2;
  monitor[6] = g_stick1+g_stick4;
  monitor[7] = g_stick2;
  monitor[8] = g_stick3;
#endif
}

/* USER CODE END 1 */
