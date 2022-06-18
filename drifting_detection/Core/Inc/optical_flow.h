#ifndef OPTICAL_FLOW_H
#define OPTICAL_FLOW_H

#include "stm32h7xx_hal.h"

#define OF_FREQ 50
#define IMG_H 64
#define IMG_W 64
#define IMG_SIZE IMG_H*IMG_W

typedef struct {
  UART_HandleTypeDef *uart;
  double prev_frame[IMG_SIZE];
  double frame[IMG_SIZE];
  double vx[IMG_SIZE]; // h * w
  double vy[IMG_SIZE]; // h * w
  double warpI2[IMG_SIZE]; // h * w * c
  double dx;
  double dy;
} optical_flow_t;

int optical_flow_init(optical_flow_t *optflw, UART_HandleTypeDef *uart);
void optical_flow_update(optical_flow_t *optflw, uint8_t *frame);

#endif /* OPTICAL_FLOW_H */
