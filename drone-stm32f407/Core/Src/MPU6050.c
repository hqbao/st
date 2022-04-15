#include "gy86.h"

#define MPU6050_I2C_ADDR 0xD0

uint8_t i2c_rx_buffer[32] = {0};

int MPU6050_init(mpu6050_t *mpu6050, I2C_HandleTypeDef *i2c,
    uint8_t data_rate, MPU6050_Accelerometer accel, MPU6050_Gyroscope gyro) {
  int counter;

  // Hold i2c
  mpu6050->i2c = i2c;

  // Update device address, 2 addresses: 0x00, 0x02. Using 0x00
  mpu6050->address = (uint8_t) MPU6050_I2C_ADDR | (uint8_t) 0x00;

  // Check device state
  for (counter = 0; counter < 5; counter += 1) {
    if (HAL_I2C_GetState(mpu6050->i2c) == HAL_I2C_STATE_READY) {
      break;
    }
  }
  if (counter >= 5) return 1;

  // Check MPU id
  uint8_t whoiam = 0x75;
  uint8_t whoiam_res;
  for (counter = 0; counter < 5; counter += 1) {
    if (HAL_I2C_Master_Transmit(mpu6050->i2c, mpu6050->address, &whoiam, 1, 100) == HAL_OK) {
      break;
    }
  }
  if (counter >= 5) return 3;

  for (counter = 0; counter < 5; counter += 1) {
    if (HAL_I2C_Master_Receive(mpu6050->i2c, mpu6050->address, &whoiam_res, 1, 100) == HAL_OK) {
      break;
    }
  }
  if (counter >= 5) return 4;

  if (whoiam_res != 0x68) return 5;

  // Wake up MPU6050
  uint8_t weakup_req[2] = {0x6B, 0x00};
  for (counter = 0; counter < 5; counter += 1) {
    if (HAL_I2C_Master_Transmit(mpu6050->i2c, mpu6050->address, weakup_req, 2, 100) == HAL_OK) {
      break;
    }
  }
  if (counter >= 5) return 6;

  // Set data rate
  uint8_t data_rate_req[2] = {0x19, data_rate};
  for (counter = 0; counter < 5; counter += 1) {
    if (HAL_I2C_Master_Transmit(mpu6050->i2c, mpu6050->address, data_rate_req, 2, 100) == HAL_OK) {
      break;
    }
  }
  if (counter >= 5) return 7;

  // Set accel config
  uint8_t accel_config = 0x1C;
  for (counter = 0; counter < 5; counter += 1) {
    if (HAL_I2C_Master_Transmit(mpu6050->i2c, mpu6050->address, &accel_config, 1, 100) == HAL_OK) {
      break;
    }
  }
  if (counter >= 5) return 8;

  for (counter = 0; counter < 5; counter += 1) {
    if (HAL_I2C_Master_Receive(mpu6050->i2c, mpu6050->address, &accel_config, 1, 100) == HAL_OK) {
      break;
    }
  }
  if (counter >= 5) return 9;

  accel_config = (accel_config & 0xE7) | (uint8_t)accel << 3;
  for (counter = 0; counter < 5; counter += 1) {
    if (HAL_I2C_Master_Transmit(mpu6050->i2c, mpu6050->address, &accel_config, 1, 100) == HAL_OK) {
      break;
    }
  }
  if (counter >= 5) return 10;

  // Set gyro config
  uint8_t gyro_config = 0x1B;
  for (counter = 0; counter < 5; counter += 1) {
    if (HAL_I2C_Master_Transmit(mpu6050->i2c, mpu6050->address, &gyro_config, 1, 100) == HAL_OK) {
      break;
    }
  }
  if (counter >= 5) return 11;

  for (counter = 0; counter < 5; counter += 1) {
    if (HAL_I2C_Master_Receive(mpu6050->i2c, mpu6050->address, &gyro_config, 1, 100) == HAL_OK) {
      break;
    }
  }
  if (counter >= 5) return 12;

  accel_config = (accel_config & 0xE7) | (uint8_t)gyro << 3;
  for (counter = 0; counter < 5; counter += 1) {
    if (HAL_I2C_Master_Transmit(mpu6050->i2c, mpu6050->address, &gyro_config, 1, 100) == HAL_OK) {
      break;
    }
  }
  if (counter >= 5) return 13;

  return 0;
}

void MPU6050_update(mpu6050_t *mpu6050) {
  // Read MPU6050
  HAL_I2C_Mem_Read(mpu6050->i2c, mpu6050->address, 0x3B, 1, i2c_rx_buffer, 14, 10);
  MPU6050_parse_6axis(mpu6050);
}

void MPU6050_parse_6axis(mpu6050_t *mpu6050) {
  // Read MPU6050
  mpu6050->ax = (int16_t)(i2c_rx_buffer[0] << 8 | i2c_rx_buffer[1]);
  mpu6050->ay = (int16_t)(i2c_rx_buffer[2] << 8 | i2c_rx_buffer[3]);
  mpu6050->az = (int16_t)(i2c_rx_buffer[4] << 8 | i2c_rx_buffer[5]);

  int16_t temp = (i2c_rx_buffer[6] << 8 | i2c_rx_buffer[7]);
  mpu6050->temperature = (float)((float)((int16_t)temp) / (float)340.0 + (float)36.53);

  mpu6050->gx = (int16_t)(i2c_rx_buffer[8] << 8 | i2c_rx_buffer[9]);
  mpu6050->gy = (int16_t)(i2c_rx_buffer[10] << 8 | i2c_rx_buffer[11]);
  mpu6050->gz = (int16_t)(i2c_rx_buffer[12] << 8 | i2c_rx_buffer[13]);
}
