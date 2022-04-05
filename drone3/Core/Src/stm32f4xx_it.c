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
#include "mpu6050.h"
#include "ms5611.h"
#include "hmc5883.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

// Motor PWM values
#define INIT_SPEED 10 // 10
#define MIN_SPEED 572 // 572
#define MAX_SPEED 772 // 1071
#define MIN_INTEGRAL -(MAX_SPEED - MIN_SPEED)*0.1
#define MAX_INTEGRAL (MAX_SPEED - MIN_SPEED)*0.1

// PID
#define P_PITCH_GAIN 16
#define I_PITCH_GAIN 0.01
#define D_PITCH_GAIN 24

#define P_ROLL_GAIN 16
#define I_ROLL_GAIN 0.01
#define D_ROLL_GAIN 24

#define P_YAW_GAIN 16
#define I_YAW_GAIN 0.01
#define D_YAW_GAIN 8

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

float g_ax_calibrated = 0.196*2000;
float g_ay_calibrated = 0.324*2000;
float g_az_calibrated = 0.347*45;

float g_gx_calibrated = -0.148*2000;
float g_gy_calibrated = -0.063*2000;
float g_gz_calibrated = 0.017*2000;

float g_ax_offset = 0;
float g_ay_offset = 0;
float g_az_offset = 0;

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

float limit(float number, float min, float max) {
  return number < min ? min : (number > max ? max : number);
}

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

extern SD_MPU6050 g_dev1;

extern MS5611 g_dev2;

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

  int16_t mx = 0, my = 0, mz = 0;

  // Read MPU6050 values
  SD_MPU6050_ReadAll(&hi2c1, &g_dev1);
  HMC5883L_getHeading(&mx, &my, &mz);

  // Norm [-1, 1]
  float gx = limit(-g_dev1.Gyroscope_X + g_gx_calibrated, -2000, 2000)/2000; // Velocity
  float gy = limit(g_dev1.Gyroscope_Y + g_gy_calibrated, -2000, 2000)/2000;
  float gz = limit(-g_dev1.Gyroscope_Z + g_gz_calibrated, -2000, 2000)/2000;
  float ax = limit(-g_dev1.Accelerometer_X + g_ax_calibrated, -2000, 2000)/2000; // Angle
  float ay = limit(-g_dev1.Accelerometer_Y + g_ay_calibrated, -2000, 2000)/2000;
  float az = limit(atan2(my, mx)*360/M_PI + g_az_calibrated, -45, 45)/45;

  // Remove noise
  g_gx = SimpleKalmanFilter_updateEstimate(&g_filters[0], gx);
  g_gy = SimpleKalmanFilter_updateEstimate(&g_filters[1], gy);
  g_gz = SimpleKalmanFilter_updateEstimate(&g_filters[2], gz);
  g_ax = SimpleKalmanFilter_updateEstimate(&g_filters[3], ax);
  g_ay = SimpleKalmanFilter_updateEstimate(&g_filters[4], ay);
  g_az = SimpleKalmanFilter_updateEstimate(&g_filters[5], az);

  // Control pitch, roll, yaw using offsets
  g_gx += g_gx_offset;
  g_gy += g_gy_offset;
  g_gz += g_gz_offset;
  g_ax += g_ax_offset;
  g_ay += g_ay_offset;
  g_az += g_az_offset;

  // Update PWM
  if (g_conn_lost_counter > 50 || g_conn_lost_counter < 0) {
    g_I_pitch = 0;
    g_I_roll = 0;
    g_I_yaw = 0;
    ctl_motors_speed(INIT_SPEED, INIT_SPEED, INIT_SPEED, INIT_SPEED);
  }
  else {
    int thrust = MIN_SPEED + g_thrust;

    g_P_pitch = g_ax*P_PITCH_GAIN;
    g_I_pitch += g_ax*I_PITCH_GAIN;
    g_I_pitch = limit(g_I_pitch, MIN_INTEGRAL, MAX_INTEGRAL);
    g_D_pitch = g_gy*D_PITCH_GAIN;

    g_P_roll = g_ay*P_ROLL_GAIN;
    g_I_roll += g_ay*I_ROLL_GAIN;
    g_I_roll = limit(g_I_roll, MIN_INTEGRAL, MAX_INTEGRAL);
    g_D_roll = g_gx*D_ROLL_GAIN;

    g_P_yaw = g_az*P_YAW_GAIN;
    g_I_yaw += g_az*I_YAW_GAIN;
    g_I_yaw = limit(g_I_yaw, MIN_INTEGRAL, MAX_INTEGRAL);
    g_D_yaw = g_gz*D_YAW_GAIN;

    g_sig1 = thrust + (g_P_pitch + g_I_pitch + g_D_pitch) + (g_P_roll + g_I_roll + g_D_roll) - (g_P_yaw + g_I_yaw + g_D_yaw);
    g_sig2 = thrust + (g_P_pitch + g_I_pitch + g_D_pitch) - (g_P_roll + g_I_roll + g_D_roll) + (g_P_yaw + g_I_yaw + g_D_yaw);
    g_sig3 = thrust - (g_P_pitch + g_I_pitch + g_D_pitch) - (g_P_roll + g_I_roll + g_D_roll) - (g_P_yaw + g_I_yaw + g_D_yaw);
    g_sig4 = thrust - (g_P_pitch + g_I_pitch + g_D_pitch) + (g_P_roll + g_I_roll + g_D_roll) + (g_P_yaw + g_I_yaw + g_D_yaw);

    g_sig1 = limit(g_sig1, MIN_SPEED, MAX_SPEED);
    g_sig2 = limit(g_sig2, MIN_SPEED, MAX_SPEED);
    g_sig3 = limit(g_sig3, MIN_SPEED, MAX_SPEED);
    g_sig4 = limit(g_sig4, MIN_SPEED, MAX_SPEED);

    ctl_motors_speed(g_sig1, g_sig2, g_sig3, g_sig4);
  }

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

  // Stop motor if lost signal from remote controller
  if (g_conn_lost_counter > 50) {
    ctl_motors_speed(INIT_SPEED, INIT_SPEED, INIT_SPEED, INIT_SPEED);
  }

  g_conn_lost_counter += 1;

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
//  send_data(g_ax, g_gx, 0,
//      g_ay, g_gy, 0,
//      g_az, g_gz, 0);
//  send_data(g_thrust, g_yaw, 0,
//      g_pitch, g_roll, 0,
//      0, 0, 0);
//  send_data(g_P_pitch, g_I_pitch, g_D_pitch,
//      g_P_roll, g_I_roll, g_D_roll,
//      g_P_yaw, g_I_yaw, g_D_yaw);
  send_data(g_sig1, g_sig2, MIN_SPEED,
      g_sig3, g_sig4, MIN_SPEED,
      0, 0, 0);

}

/* USER CODE END 1 */
