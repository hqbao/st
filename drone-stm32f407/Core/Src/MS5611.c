#include "gy86.h"
#include <math.h>
#include "stm32f4xx_it.h"

#define MS5611_ADDR 0x77
#define CMD_RESET 0x1E
#define CMD_PROM_C0 0xA0
#define CMD_PROM_C1 0xA2
#define CMD_PROM_C2 0xA4
#define CMD_PROM_C3 0xA6
#define CMD_PROM_C4 0xA8
#define CMD_PROM_C5 0xAA
#define CMD_PROM_C6 0xAC
#define CMD_PROM_C7 0xAE

#define PRESSURE_OSR_256 0x40
#define PRESSURE_OSR_512 0x42
#define PRESSURE_OSR_1024 0x44
#define PRESSURE_OSR_2048 0x46
#define PRESSURE_OSR_4096 0x48

#define TEMP_OSR_256 0x50
#define TEMP_OSR_512 0x52
#define TEMP_OSR_1024 0x54
#define TEMP_OSR_2048 0x56
#define TEMP_OSR_4096 0x58

void _reset(ms5611_t *ms5611) {
  ms5611->tx = CMD_RESET;
  HAL_I2C_Master_Transmit(ms5611->i2c, MS5611_ADDR << 1 , &ms5611->tx, 1, 100);
  HAL_Delay(10);

  // For Temperature > 20 Celsius
  ms5611->T2 = 0;
  ms5611->OFF2 = 0;
  ms5611->SENS2 = 0;
}

void _read_PROM(ms5611_t *ms5611) {
  ms5611->tx = CMD_PROM_C0;
  HAL_I2C_Master_Transmit(ms5611->i2c, MS5611_ADDR << 1, &ms5611->tx, 1, 100);
  HAL_I2C_Master_Receive(ms5611->i2c, MS5611_ADDR << 1 , ms5611->rx_buf, 2, 100);
  ms5611->C[0] = ms5611->rx_buf[0] << 8 | ms5611->rx_buf[1];
  HAL_Delay(10);

  ms5611->tx = CMD_PROM_C1;
  HAL_I2C_Master_Transmit(ms5611->i2c, MS5611_ADDR << 1, &ms5611->tx, 1, 100);
  HAL_I2C_Master_Receive(ms5611->i2c, MS5611_ADDR << 1 , ms5611->rx_buf, 2, 100);
  ms5611->C[1] = ms5611->rx_buf[0] << 8 | ms5611->rx_buf[1];
  HAL_Delay(10);

  ms5611->tx = CMD_PROM_C2;
  HAL_I2C_Master_Transmit(ms5611->i2c, MS5611_ADDR << 1, &ms5611->tx, 1, 100);
  HAL_I2C_Master_Receive(ms5611->i2c, MS5611_ADDR << 1 , ms5611->rx_buf, 2, 100);
  ms5611->C[2] = ms5611->rx_buf[0] << 8 | ms5611->rx_buf[1];
  HAL_Delay(10);

  ms5611->tx = CMD_PROM_C3;
  HAL_I2C_Master_Transmit(ms5611->i2c, MS5611_ADDR << 1, &ms5611->tx, 1, 100);
  HAL_I2C_Master_Receive(ms5611->i2c, MS5611_ADDR << 1 , ms5611->rx_buf, 2, 100);
  ms5611->C[3] = ms5611->rx_buf[0] << 8 | ms5611->rx_buf[1];
  HAL_Delay(10);

  ms5611->tx = CMD_PROM_C4;
  HAL_I2C_Master_Transmit(ms5611->i2c, MS5611_ADDR << 1, &ms5611->tx, 1, 100);
  HAL_I2C_Master_Receive(ms5611->i2c, MS5611_ADDR << 1 , ms5611->rx_buf, 2, 100);
  ms5611->C[4] = ms5611->rx_buf[0] << 8 | ms5611->rx_buf[1];
  HAL_Delay(10);

  ms5611->tx = CMD_PROM_C5;
  HAL_I2C_Master_Transmit(ms5611->i2c, MS5611_ADDR << 1, &ms5611->tx, 1, 100);
  HAL_I2C_Master_Receive(ms5611->i2c, MS5611_ADDR << 1 , ms5611->rx_buf, 2, 100);
  ms5611->C[5] = ms5611->rx_buf[0] << 8 | ms5611->rx_buf[1];
  HAL_Delay(10);

  ms5611->tx = CMD_PROM_C6;
  HAL_I2C_Master_Transmit(ms5611->i2c, MS5611_ADDR << 1, &ms5611->tx, 1, 100);
  HAL_I2C_Master_Receive(ms5611->i2c, MS5611_ADDR << 1 , ms5611->rx_buf, 2, 100);
  ms5611->C[6] = ms5611->rx_buf[0] << 8 | ms5611->rx_buf[1];
  HAL_Delay(10);

  ms5611->tx = CMD_PROM_C7;
  HAL_I2C_Master_Transmit(ms5611->i2c, MS5611_ADDR << 1, &ms5611->tx, 1, 100);
  HAL_I2C_Master_Receive(ms5611->i2c, MS5611_ADDR << 1 , ms5611->rx_buf, 2, 100);
  ms5611->C[7] = ms5611->rx_buf[0] << 8 | ms5611->rx_buf[1];
  HAL_Delay(10);
}

int MS5611_init(ms5611_t *ms5611, I2C_HandleTypeDef *i2c) {
  ms5611->i2c = i2c;

  _reset(ms5611);
  _read_PROM(ms5611);

  return 0;
}

void MS5611_req_temperature(ms5611_t *ms5611, OSR osr) {
  ms5611->tx = TEMP_OSR_256 + (2 * osr);
  HAL_I2C_Master_Transmit(ms5611->i2c, MS5611_ADDR << 1, &ms5611->tx, 1, 100);
}

void MS5611_req_pressure(ms5611_t *ms5611, OSR osr) {
  ms5611->tx = PRESSURE_OSR_256 + (2 * osr);
  HAL_I2C_Master_Transmit(ms5611->i2c, MS5611_ADDR << 1, &ms5611->tx, 1, 100);
}

void MS5611_read_temperature(ms5611_t *ms5611) {
  //Read ADC
  ms5611->tx = 0x00;
  HAL_I2C_Master_Transmit(ms5611->i2c, MS5611_ADDR << 1, &ms5611->tx, 1, 100);
  HAL_I2C_Master_Receive(ms5611->i2c, (MS5611_ADDR << 1) | 0x01, ms5611->rx_temp, 3, 100);
//  HAL_I2C_Mem_Read(ms5611->i2c, MS5611_ADDR <<1 , 0x00, 1, ms5611->rx_temp, 3, 100);

  ms5611->digi_tem_D2 = (ms5611->rx_temp[0] << 16) | (ms5611->rx_temp[1] << 8) | ms5611->rx_temp[2];
}

void MS5611_read_pressure(ms5611_t *ms5611) {
  //Read ADC
  ms5611->tx = 0x00;
  HAL_I2C_Master_Transmit(ms5611->i2c, MS5611_ADDR << 1, &ms5611->tx, 1, 100);
  HAL_I2C_Master_Receive(ms5611->i2c, (MS5611_ADDR << 1) | 0x01, ms5611->rx_press, 3, 100);
  //HAL_I2C_Mem_Read(ms5611->i2c, MS5611_ADDR <<1, 0x00, 1, ms5611->rx_press, 3, 100);

  ms5611->digi_psr_D1 = ms5611->rx_press[0] << 16 | ms5611->rx_press[1] << 8 | ms5611->rx_press[2];
}

void MS5611_calc_temperature(ms5611_t *ms5611) {
  ms5611->dT = ms5611->C[5];
  ms5611->dT <<= 8; // Calculated up to C5 * 2^8
  ms5611->dT *= -1; // Apply negative sign
  ms5611->dT += ms5611->digi_tem_D2; // = D2 - C5 * 2^8

  ms5611->TEMP = ms5611->dT * ms5611->C[6];
  ms5611->TEMP >>= 23; // Calculated up to dT * C6 / 2^23
  ms5611->TEMP += 2000;
}

void MS5611_calc_pressure(ms5611_t *ms5611) {
  ms5611->OFF = ms5611->C[2];
  ms5611->OFF <<= 16; // Calculated up to C2 * 2^16
  ms5611->OFF += (ms5611->C[4] * ms5611->dT) >> 7;


  ms5611->SENS = ms5611->C[1];
  ms5611->SENS <<= 15; // Calculated up to C1 * 2^15
  ms5611->SENS += (ms5611->C[3] * ms5611->dT) >>8;

  ms5611->P = ((ms5611->digi_psr_D1 * ms5611->SENS) / pow(2, 21) - ms5611->OFF) / pow(2, 15);
}

#define SEA_PRESSURE 1013.25f
void MS5611_calc_altitude(ms5611_t *ms5611) {
  float temperature = (float)ms5611->TEMP/100.f;
  float pressure = (float)ms5611->P/100.f;
  ms5611->altitude = (1.0f - powf((pressure / SEA_PRESSURE), 0.1902226f)) * (temperature + 273.15f) / 0.0065f;
}

// Counter checks depend on timer frequency
#define STEP_1 0
#define STEP_2 2
#define STEP_3 4
#define STEP_END 6
void MS5611_update(ms5611_t *ms5611) {
  static int counter = 0;

  if (counter == STEP_1) {
    MS5611_req_temperature(ms5611, OSR_4096);
  }

  if (counter == STEP_2) {
    MS5611_read_temperature(ms5611);
    MS5611_calc_temperature(ms5611);
    MS5611_req_pressure(ms5611, OSR_4096);
  }

  if (counter == STEP_3) {
    MS5611_read_pressure(ms5611);
    MS5611_calc_pressure(ms5611);
    MS5611_calc_altitude(ms5611);
  }

  counter += 1;
  if (counter >= STEP_END)
    counter = 0;
}
