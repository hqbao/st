/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32h7xx_it.c
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
#include "stm32h7xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdio.h>
#include <string.h>
#include "OV7670.h"
#include "color_conversion.h"
#include "Coarse2FineFlowWrapper.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

uint8_t g_uart_rx_buffer[1] = {0};
static uint8_t g_prev_img = 0;
static uint16_t g_height = 0;
static uint16_t g_width = 0;
static double g_img0[4800];
static double g_img1[4800];
static double g_v1[4800];
static double g_v2[4800];
static double g_warpI2[4800];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

void log_string(const char *str);
void log_data(const uint8_t *data, uint16_t size);
void schedule_50hz(void);
void schedule_5hz(void);
void update_image(void);
void update_flow(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_dcmi;
extern DCMI_HandleTypeDef hdcmi;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim7;
extern UART_HandleTypeDef huart1;
/* USER CODE BEGIN EV */

extern uint32_t g_image_data[144*352];

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
  HAL_DMA_IRQHandler(&hdma_dcmi);
  /* USER CODE BEGIN DMA1_Stream0_IRQn 1 */

  /* USER CODE END DMA1_Stream0_IRQn 1 */
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
  * @brief This function handles TIM6 global interrupt, DAC1_CH1 and DAC1_CH2 underrun error interrupts.
  */
void TIM6_DAC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM6_DAC_IRQn 0 */

  schedule_50hz();

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

  schedule_5hz();

  /* USER CODE END TIM7_IRQn 0 */
  HAL_TIM_IRQHandler(&htim7);
  /* USER CODE BEGIN TIM7_IRQn 1 */

  /* USER CODE END TIM7_IRQn 1 */
}

/**
  * @brief This function handles DCMI global interrupt.
  */
void DCMI_IRQHandler(void)
{
  /* USER CODE BEGIN DCMI_IRQn 0 */

  /* USER CODE END DCMI_IRQn 0 */
  HAL_DCMI_IRQHandler(&hdcmi);
  /* USER CODE BEGIN DCMI_IRQn 1 */

  /* USER CODE END DCMI_IRQn 1 */
}

/* USER CODE BEGIN 1 */

void log_string(const char *str) {
  HAL_UART_Transmit_IT(&huart1, (uint8_t*)str, (uint16_t)strlen(str));
}

void log_data(const uint8_t *data, uint16_t size) {
  HAL_UART_Transmit_IT(&huart1, (uint8_t*)data, (uint16_t)size);
}

void schedule_50hz(void) {

}

void schedule_5hz(void) {
  update_image();
  update_flow();
  HAL_UART_Receive_IT(&huart1, g_uart_rx_buffer, 1);
}

void update_image(void) {
  static uint8_t format;
  OV7670_getImageInfo(&g_width, &g_height, &format);

  double *img_ptr = g_prev_img == 0 ? g_img1 : g_img0;

  for (int i = 0; i < g_height; i += 1) {
    for (int j = 0; j < (int)g_width/2; j += 1) {
      int idx = i*(g_width/2) + j;
      uint32_t temp = g_image_data[idx];
      if (format == YUV422) {
        int16_t Y2 = (temp >> 24) & 0x00FF;
        int16_t U = ((temp >> 16) & 0x00FF) - 128;
        int16_t Y1 = (temp >> 8) & 0x00FF;
        int16_t V = (temp & 0x00FF) - 128;
        uint32_t pix = YUVtoRGB888(Y1, U, V);
        uint32_t next_pix = YUVtoRGB888(Y2, U, V);

        uint8_t r = (pix >> (0)) & 0xff;
        uint8_t g = (pix >> (8)) & 0xff;
        uint8_t b = (pix >> (16)) & 0xff;
        uint8_t rgb = ((r + g + b)/3);
        img_ptr[2*idx] = rgb;

        r = (next_pix >> (0)) & 0xff;
        g = (next_pix >> (8)) & 0xff;
        b = (next_pix >> (16)) & 0xff;
        rgb = ((r + g + b)/3);
        img_ptr[2*idx + 1] = rgb;
      }
      else {
        uint16_t rbg1 = (temp >> 0) & 0x00FF;
        uint16_t rbg2 = (temp >> 16) & 0x00FF;
        uint32_t pix = RGB565toRGB888(rbg1);
        uint32_t next_pix = RGB565toRGB888(rbg2);

        uint8_t r = (pix >> (0)) & 0xff;
        uint8_t g = (pix >> (8)) & 0xff;
        uint8_t b = (pix >> (16)) & 0xff;
        uint8_t rgb = ((r + g + b)/3);
        img_ptr[2*idx] = rgb;

        r = (next_pix >> (0)) & 0xff;
        g = (next_pix >> (8)) & 0xff;
        b = (next_pix >> (16)) & 0xff;
        rgb = ((r + g + b)/3);
        img_ptr[2*idx + 1] = rgb;
      }
    }
  }

  g_prev_img = g_prev_img == 0 ? 1 : 0;
}

void update_flow(void) {
//  Coarse2FineFlowWrapper(g_v1, g_v2, g_warpI2, g_img0, g_img1,
//      0.0012, 3/4, 30, 2, 1, 1, 1, 60, 80, 1);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_1);
  static uint8_t img[4800];
  for (int i = 0; i < g_height*g_width; i +=1)
    img[i] = (uint8_t)g_img0[i];

  log_data(img, g_height*g_width);
}

/* USER CODE END 1 */
