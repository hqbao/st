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
#include "MPU6050.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

// PID
#define P_GAIN 0.0005 // 0.0005
#define I_GAIN 0.00001
#define D_GAIN 0.003 // 0.005

// Motor PWM values
#define INIT_SPEED 10 // 10
#define MIN_SPEED 143 // 140
#define MAX_SPEED 243 // 540
#define MIN_INTEGRAL -(MAX_SPEED - MIN_SPEED)*0.1
#define MAX_INTEGRAL (MAX_SPEED - MIN_SPEED)*0.1

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

// Accel, gyro
float g_gx = 0;
float g_gy = 0;
float g_gz = 0;
float g_ax = 0;
float g_ay = 0;
float g_az = 0;

float g_ax_offset = 0;
float g_ay_offset = 0;
float g_az_offset = 0; // No use

float g_gx_offset = 0;
float g_gy_offset = 0;
float g_gz_offset = 0;

// PID
float g_P_pitch = 0;
float g_I_pitch = 0;
float g_D_pitch = 0;

float g_P_roll = 0;
float g_I_roll = 0;
float g_D_roll = 0;

float g_P_yaw = 0;
float g_I_yaw = 0;
float g_D_yaw = 0;

float g_sig1 = 0;
float g_sig2 = 0;
float g_sig3 = 0;
float g_sig4 = 0;

uint8_t g_thrust = 0;
int8_t g_pitch = 0;
int8_t g_roll = 0;
int8_t g_yaw = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

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
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern UART_HandleTypeDef huart1;
/* USER CODE BEGIN EV */

extern I2C_HandleTypeDef hi2c1;

extern kalman_filter g_filters[14];

MPU6050_t g_dev1;

// Remote control
extern int g_conn_lost_counter;
extern uint8_t g_control[5];

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
  * @brief This function handles TIM3 global interrupt.
  */
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */

//  // Read MPU6050 values
//  MPU6050_Read_All(&hi2c1, &g_dev1);
//  MPU6050_Parsing_NoOffest(&g_dev1);
//  float gx = -g_dev1.Gx + g_gx_offset; // Velocity
//  float gy = g_dev1.Gy + g_gy_offset;
//  float gz = -g_dev1.Gz + g_gz_offset;
//  float ax = -g_dev1.Ax + g_ax_offset; // Angle
//  float ay = -g_dev1.Ay + g_ay_offset;
//  float az = atan2(g_dev1.Mag_Y_RAW, g_dev1.Mag_X_RAW)*180/M_PI + g_az_offset;
//
//  // Remove noise
//  g_gx = SimpleKalmanFilter_updateEstimate(&g_filters[0], gx);
//  g_gy = SimpleKalmanFilter_updateEstimate(&g_filters[1], gy);
//  g_gz = SimpleKalmanFilter_updateEstimate(&g_filters[2], gz);
//  g_ax = SimpleKalmanFilter_updateEstimate(&g_filters[3], ax);
//  g_ay = SimpleKalmanFilter_updateEstimate(&g_filters[4], ay);
//  g_az = SimpleKalmanFilter_updateEstimate(&g_filters[5], az);
//
//  // Update PWM
//  if (g_conn_lost_counter > 50 || g_conn_lost_counter < 0) {
//    g_I_pitch = 0;
//    g_I_roll = 0;
//    g_I_yaw = 0;
//    ctl_motors_speed(INIT_SPEED, INIT_SPEED, INIT_SPEED, INIT_SPEED);
//  }
//  else {
//    int thrust = MIN_SPEED + g_thrust;
//
//    g_P_pitch = g_ax*P_GAIN;
//    g_I_pitch += g_ax*I_GAIN;
//    g_I_pitch = g_I_pitch > MAX_INTEGRAL ? MAX_INTEGRAL : (g_I_pitch < MIN_INTEGRAL ? MIN_INTEGRAL : g_I_pitch);
//    g_D_pitch = g_gy*D_GAIN;
//
//    g_P_roll = g_ay*P_GAIN;
//    g_I_roll += g_ay*I_GAIN;
//    g_I_roll = g_I_roll > MAX_INTEGRAL ? MAX_INTEGRAL : (g_I_roll < MIN_INTEGRAL ? MIN_INTEGRAL : g_I_roll);
//    g_D_roll = g_gx*D_GAIN;
//
//    g_P_yaw = g_az*P_GAIN;
//    g_I_yaw += g_az*I_GAIN;
//    g_I_yaw = g_I_yaw > MAX_INTEGRAL ? MAX_INTEGRAL : (g_I_yaw < MIN_INTEGRAL ? MIN_INTEGRAL : g_I_yaw);
//    g_D_yaw = g_gz*D_GAIN;
//
//    g_sig1 = thrust + (g_P_pitch + g_I_pitch + g_D_pitch) + (g_P_roll + g_I_roll + g_D_roll) - (g_P_yaw + g_I_yaw + g_D_yaw);
//    g_sig2 = thrust + (g_P_pitch + g_I_pitch + g_D_pitch) - (g_P_roll + g_I_roll + g_D_roll) + (g_P_yaw + g_I_yaw + g_D_yaw);
//    g_sig3 = thrust - (g_P_pitch + g_I_pitch + g_D_pitch) - (g_P_roll + g_I_roll + g_D_roll) - (g_P_yaw + g_I_yaw + g_D_yaw);
//    g_sig4 = thrust - (g_P_pitch + g_I_pitch + g_D_pitch) + (g_P_roll + g_I_roll + g_D_roll) + (g_P_yaw + g_I_yaw + g_D_yaw);
//
//    g_sig1 = g_sig1 > MAX_SPEED ? MAX_SPEED : (g_sig1 < MIN_SPEED ? MIN_SPEED : g_sig1);
//    g_sig2 = g_sig2 > MAX_SPEED ? MAX_SPEED : (g_sig2 < MIN_SPEED ? MIN_SPEED : g_sig2);
//    g_sig3 = g_sig3 > MAX_SPEED ? MAX_SPEED : (g_sig3 < MIN_SPEED ? MIN_SPEED : g_sig3);
//    g_sig4 = g_sig4 > MAX_SPEED ? MAX_SPEED : (g_sig4 < MIN_SPEED ? MIN_SPEED : g_sig4);
//
//    ctl_motors_speed(g_sig1, g_sig2, g_sig3, g_sig4);
//  }

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

/* USER CODE BEGIN 1 */

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
  // To know whether this timer is hanging
  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_7);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  // Update alive status
  g_conn_lost_counter = 0;

  // To know whether this timer is hanging
  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_6);

  // Serialize control values
  uint8_t g_control_1st_idx = 0;
  if (g_control[0] == 254) g_control_1st_idx = 1;
  if (g_control[1] == 254) g_control_1st_idx = 2;
  if (g_control[2] == 254) g_control_1st_idx = 3;
  if (g_control[3] == 254) g_control_1st_idx = 4;
  if (g_control[4] == 254) g_control_1st_idx = 0;
  g_thrust = g_control[g_control_1st_idx];
  g_pitch = g_control[(g_control_1st_idx+1)%5]-100;
  g_roll = g_control[(g_control_1st_idx+2)%5]-100;
  g_yaw = g_control[(g_control_1st_idx+3)%5]-100;

  // Update monitor
  monitor(g_ax, g_gx, 0,
      g_ay, g_gy, 0,
      g_az, g_gz, 0);
//  monitor(g_thrust, g_yaw, 0,
//      g_pitch, g_roll, 0,
//      0, 0, 0);
//  monitor(g_P_pitch, g_I_pitch, g_D_pitch,
//      g_P_roll, g_I_roll, g_D_roll,
//      0, 0, g_D_yaw);
//  monitor(g_sig1, g_sig2, MIN_SPEED,
//      g_sig3, g_sig4, MIN_SPEED,
//      0, 0, 0);

}

/* USER CODE END 1 */
