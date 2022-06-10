#ifndef KALMAN_H
#define KALMAN_H

#include "stm32h7xx_hal.h"

typedef struct {
  SPI_HandleTypeDef *spi;
  int8_t motion;
  int8_t dx;
  int8_t dy;
} adns3080_t;

int adns3080_init(adns3080_t *of, SPI_HandleTypeDef *spi);
void adns3080_update(adns3080_t *of);

#endif
