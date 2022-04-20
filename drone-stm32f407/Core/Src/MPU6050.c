#include "gy86.h"
#include <string.h>
#include <stdlib.h>

#define MPU6050_I2C_ADDR 0xD0

#define FREQ 200
#define SSF_GYRO 65.5
#define X 0
#define Y 1
#define Z 2
#define YAW 0
#define PITCH 1
#define ROLL 2

uint8_t i2c_rx_buffer[32] = {0};

// https://github.com/lobodol/IMU/blob/master/imu.ino?fbclid=IwAR1bmIn_qUKA1KbK5g8u7M5T1lKf2K4e0y23TLkXcwpcrFv7rZ7KPQ2Gsvo
void calc_angles(mpu6050_t *mpu6050) {
  // Angle calculation using integration
  mpu6050->gyro_angle[X] += mpu6050->gx / FREQ;
  mpu6050->gyro_angle[Y] += -mpu6050->gy / FREQ; // Change sign to match the accelerometer's one

  // Transfer roll to pitch if IMU has yawed
  mpu6050->gyro_angle[Y] += mpu6050->gyro_angle[X] * sin(mpu6050->gz * (M_PI / (FREQ * 180)));
  mpu6050->gyro_angle[X] -= mpu6050->gyro_angle[Y] * sin(mpu6050->gz * (M_PI / (FREQ * 180)));

  // Calculate total 3D acceleration vector : √(X² + Y² + Z²)
  mpu6050->acc_total_vector = sqrt(pow(mpu6050->ax, 2) + pow(mpu6050->ay, 2) + pow(mpu6050->az, 2));

  // To prevent asin to produce a NaN, make sure the input value is within [-1;+1]
  if (abs(mpu6050->ax) < mpu6050->acc_total_vector) {
    mpu6050->acc_angle[X] = asin((float)mpu6050->ay / mpu6050->acc_total_vector) * (180 / M_PI); // asin gives angle in radian. Convert to degree multiplying by 180/pi
  }

  if (abs(mpu6050->ay) < mpu6050->acc_total_vector) {
    mpu6050->acc_angle[Y] = asin((float)mpu6050->ax / mpu6050->acc_total_vector) * (180 / M_PI);
  }

  if (mpu6050->initialized == 1) {
    // Correct the drift of the gyro with the accelerometer
    mpu6050->gyro_angle[X] = mpu6050->gyro_angle[X] * 0.5 + mpu6050->acc_angle[X] * 0.5;
    mpu6050->gyro_angle[Y] = mpu6050->gyro_angle[Y] * 0.5 + mpu6050->acc_angle[Y] * 0.5;
  }
  else {
    // At very first start, init gyro angles with accelerometer angles
    mpu6050->gyro_angle[X] = mpu6050->acc_angle[X];
    mpu6050->gyro_angle[Y] = mpu6050->acc_angle[Y];

    mpu6050->initialized = 1;
  }

  // To dampen the pitch and roll angles a complementary filter is used
  mpu6050->measures[ROLL] = mpu6050-> measures[ROLL] * 0.9 + mpu6050->gyro_angle[X] * 0.1;
  mpu6050->measures[PITCH] = mpu6050->measures[PITCH] * 0.9 + mpu6050->gyro_angle[Y] * 0.1;
  mpu6050->measures[YAW] = -mpu6050->gz; // Store the angular motion for this axis

  // Norm [-1, 1]
  mpu6050->angle_x = -mpu6050->measures[PITCH];
  mpu6050->angle_y = -mpu6050->measures[ROLL];
  mpu6050->angle_z -= mpu6050->measures[YAW]*0.001;
}

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

  kalman_filter_init(&mpu6050->kf[0], 2, 2, 0.01); // Accel x
  kalman_filter_init(&mpu6050->kf[1], 2, 2, 0.01); // Accel y
  kalman_filter_init(&mpu6050->kf[2], 2, 2, 0.01); // Accel z
  kalman_filter_init(&mpu6050->kf[3], 2, 2, 0.01); // Gyro x
  kalman_filter_init(&mpu6050->kf[4], 2, 2, 0.01); // Gyro Y
  kalman_filter_init(&mpu6050->kf[5], 2, 2, 0.01); // Gyro z

  // For angle calculation
  memset(mpu6050->gyro_angle, 0, 3 * sizeof(float));
  memset(mpu6050->acc_angle, 0, 3 * sizeof(float));
  memset(mpu6050->measures, 0, 3 * sizeof(float));
  mpu6050->initialized = 0;
  mpu6050->angle_z = 0;

  return 0;
}

void MPU6050_set_offset(mpu6050_t *mpu6050,
    float ax_offset, float ay_offset, float az_offset,
    float gx_offset, float gy_offset, float gz_offset) {
  mpu6050->ax_offset = ax_offset;
  mpu6050->ay_offset = ay_offset;
  mpu6050->az_offset = az_offset;
  mpu6050->gx_offset = gx_offset;
  mpu6050->gy_offset = gy_offset;
  mpu6050->gz_offset = gz_offset;
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

  mpu6050->gx = -(int16_t)(i2c_rx_buffer[8] << 8 | i2c_rx_buffer[9]);
  mpu6050->gy = (int16_t)(i2c_rx_buffer[10] << 8 | i2c_rx_buffer[11]);
  mpu6050->gz = -(int16_t)(i2c_rx_buffer[12] << 8 | i2c_rx_buffer[13]);

  mpu6050->gx = mpu6050->gx / SSF_GYRO;
  mpu6050->gy = mpu6050->gy / SSF_GYRO;
  mpu6050->gz = mpu6050->gz / SSF_GYRO;

  mpu6050->ax = kalman_filter_update(&mpu6050->kf[0], mpu6050->ax) + mpu6050->ax_offset;
  mpu6050->ay = kalman_filter_update(&mpu6050->kf[1], mpu6050->ay) + mpu6050->ay_offset;
  mpu6050->az = kalman_filter_update(&mpu6050->kf[2], mpu6050->az) + mpu6050->az_offset;
  mpu6050->gx = kalman_filter_update(&mpu6050->kf[3], mpu6050->gx) + mpu6050->gx_offset;
  mpu6050->gy = kalman_filter_update(&mpu6050->kf[4], mpu6050->gy) + mpu6050->gy_offset;
  mpu6050->gz = kalman_filter_update(&mpu6050->kf[5], mpu6050->gz) + mpu6050->gz_offset;

  calc_angles(mpu6050);
}
