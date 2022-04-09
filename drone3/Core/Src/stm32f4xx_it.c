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

#define MONITOR 2 // 1: 6 axis, 2: PID, 3: ESC, 4: PRY

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

// Motor PWM values
#define INIT_SPEED 15 // 10
#define MIN_SPEED 547 // 572
#define MAX_SPEED 847 // 1071
#define MIN_INTEGRAL -(MAX_SPEED - MIN_SPEED)*0.2
#define MAX_INTEGRAL (MAX_SPEED - MIN_SPEED)*0.2

// PID
#define P_PITCH_GAIN 50 // 16
#define I_PITCH_GAIN 0.02 // 0.01
#define D_PITCH_GAIN 20 // 24

#define P_ROLL_GAIN 50 // 16
#define I_ROLL_GAIN 0.02 // 0.01
#define D_ROLL_GAIN 20 // 24

#define P_YAW_GAIN 30
#define I_YAW_GAIN 0.01
#define D_YAW_GAIN 10

#define MAX_LOST_CONN_COUNTER 50

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

// Accel, gyro
float g_ax = 0;
float g_ay = 0;
float g_az = 0;
float g_gx = 0;
float g_gy = 0;
float g_gz = 0;

// Min values from calibration
float g_min_ax = -5000;
float g_min_ay = -5000;
float g_min_gx = -2000;
float g_min_gy = -2000;
float g_min_gz = -2000;

// Max values from calibration
float g_max_ax = 5000;
float g_max_ay = 5000;
float g_max_gx = 2000;
float g_max_gy = 2000;
float g_max_gz = 2000;

int16_t g_mx = 0;
int16_t g_my = 0;
int16_t g_mz = 0;

// Offset values after calibration
float g_ax_offset = 422;
float g_ay_offset = 783;
float g_az_offset = -618;
float g_gx_offset = -288;
float g_gy_offset = -128;
float g_gz_offset = 32;

float g_az_current = 0;

// Take off
FlyMode fly_mode = init;

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

// Remote control
int32_t pwm_in[16];
float g_thrust = 0;
float g_pitch = 0;
float g_roll = 0;
float g_yaw = 0;

// UART control
uint8_t g_uart_thrust = 0;
int8_t g_uart_pitch = 0;
int8_t g_uart_roll = 0;
int8_t g_uart_yaw = 0;

// Monitor
float monitor[9] = {0};

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

uint8_t calibrated() {
  return 1;
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

  // Read MPU6050 values
  SD_MPU6050_ReadAll(&hi2c1, &g_dev1);
  HMC5883L_getHeading(&g_mx, &g_my, &g_mz);

  if (!calibrated()) {
    float ax = -g_dev1.Accelerometer_X + g_ax_offset;
    float ay = -g_dev1.Accelerometer_Y + g_ay_offset;
    float az = atan2(g_my, g_mx);
    float gx = -g_dev1.Gyroscope_X + g_gx_offset;
    float gy = g_dev1.Gyroscope_Y + g_gy_offset;
    float gz = -g_dev1.Gyroscope_Z + g_gz_offset;

    g_ax = SimpleKalmanFilter_updateEstimate(&g_filters[0], ax);
    g_ay = SimpleKalmanFilter_updateEstimate(&g_filters[1], ay);
    g_az = SimpleKalmanFilter_updateEstimate(&g_filters[2], az);
    g_gx = SimpleKalmanFilter_updateEstimate(&g_filters[3], gx);
    g_gy = SimpleKalmanFilter_updateEstimate(&g_filters[4], gy);
    g_gz = SimpleKalmanFilter_updateEstimate(&g_filters[5], gz);
  }
  else {
    // Norm [-1, 1]
    float ax = limit(-g_dev1.Accelerometer_X + g_ax_offset, g_min_ax, g_max_ax) / (0.5*(g_max_ax - g_min_ax)); // Angle
    float ay = limit(-g_dev1.Accelerometer_Y + g_ay_offset, g_min_ay, g_max_ay) / (0.5*(g_max_ay - g_min_ay));
    float az = atan2(g_my, g_mx);
    float gx = limit(-g_dev1.Gyroscope_X + g_gx_offset, g_min_gx, g_max_gx) / (0.5*(g_max_gx - g_min_gx)); // Velocity
    float gy = limit(g_dev1.Gyroscope_Y + g_gy_offset, g_min_gy, g_max_gy) / (0.5*(g_max_gy - g_min_gy));
    float gz = limit(-g_dev1.Gyroscope_Z + g_gz_offset, g_min_gz, g_max_gz) / (0.5*(g_max_gz - g_min_gz));

    // Remove noise
    g_ax = SimpleKalmanFilter_updateEstimate(&g_filters[0], ax);
    g_ay = SimpleKalmanFilter_updateEstimate(&g_filters[1], ay);
    g_az = SimpleKalmanFilter_updateEstimate(&g_filters[2], az);
    g_gx = SimpleKalmanFilter_updateEstimate(&g_filters[3], gx);
    g_gy = SimpleKalmanFilter_updateEstimate(&g_filters[4], gy);
    g_gz = SimpleKalmanFilter_updateEstimate(&g_filters[5], gz);
  }

  // Control pitch, roll, yaw using offsets
  g_ax += -0.02*g_pitch;
  g_ay += 0.02*g_roll;
  g_az += -0.02*g_yaw;

  if (g_thrust <= -99 && g_yaw <= -99
      && g_pitch <= -99 && g_roll >= 98) {
    fly_mode = ready;
  }

  g_conn_lost_counter += 1;
  if (g_conn_lost_counter > MAX_LOST_CONN_COUNTER || g_conn_lost_counter < 0) {
    g_I_pitch = 0;
    g_I_roll = 0;
    g_I_yaw = 0;
    ctl_motors_speed(INIT_SPEED, INIT_SPEED, INIT_SPEED, INIT_SPEED);
    fly_mode = init;
  }

  switch (fly_mode) {
    case init:
      g_I_pitch = 0;
      g_I_roll = 0;
      g_I_yaw = 0;

      ctl_motors_speed(INIT_SPEED, INIT_SPEED, INIT_SPEED, INIT_SPEED);
      break;
    case ready:
      g_I_pitch = 0;
      g_I_roll = 0;
      g_I_yaw = 0;

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

      int thrust = MIN_SPEED + 2*g_thrust;

      g_sig1 = thrust + (g_P_pitch + g_I_pitch + g_D_pitch) + (g_P_roll + g_I_roll + g_D_roll) - (g_P_yaw + g_I_yaw + g_D_yaw);
      g_sig2 = thrust + (g_P_pitch + g_I_pitch + g_D_pitch) - (g_P_roll + g_I_roll + g_D_roll) + (g_P_yaw + g_I_yaw + g_D_yaw);
      g_sig3 = thrust - (g_P_pitch + g_I_pitch + g_D_pitch) - (g_P_roll + g_I_roll + g_D_roll) - (g_P_yaw + g_I_yaw + g_D_yaw);
      g_sig4 = thrust - (g_P_pitch + g_I_pitch + g_D_pitch) + (g_P_roll + g_I_roll + g_D_roll) + (g_P_yaw + g_I_yaw + g_D_yaw);

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
  monitor[0] = g_ax;
  monitor[1] = g_ay;
  monitor[2] = g_az;
  monitor[3] = g_gx;
  monitor[4] = g_gy;
  monitor[5] = g_gz;
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
  monitor[2] = MIN_SPEED - 20;
  monitor[3] = g_sig3;
  monitor[4] = g_sig4;
  monitor[5] = MIN_SPEED - 20;
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
  g_uart_thrust = g_control[g_control_1st_idx];
  g_uart_pitch = g_control[(g_control_1st_idx+1)%5]-100;
  g_uart_roll = g_control[(g_control_1st_idx+2)%5]-100;
  g_uart_yaw = g_control[(g_control_1st_idx+3)%5]-100;

  // Update monitor
  send_data(monitor[0], monitor[1], monitor[2],
      monitor[3], monitor[4], monitor[5],
      monitor[6], monitor[7], monitor[8]);
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
  if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
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
  }

  if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) {
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
  }

  if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3) {
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
  }

  if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4) {
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
  }

  g_thrust = SimpleKalmanFilter_updateEstimate(&g_filters[6], pwm_in[5] - 300);
  g_yaw = SimpleKalmanFilter_updateEstimate(&g_filters[7], pwm_in[2] - 300);
  g_pitch = SimpleKalmanFilter_updateEstimate(&g_filters[8], pwm_in[8] - 300);
  g_roll = SimpleKalmanFilter_updateEstimate(&g_filters[9], pwm_in[11] - 300);

#if MONITOR == 4
  monitor[0] = g_thrust;
  monitor[1] = g_yaw;
  monitor[2] = 0;
  monitor[3] = g_pitch;
  monitor[4] = g_roll;
  monitor[5] = 0;
#endif
}

/* USER CODE END 1 */
