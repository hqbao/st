#ifndef GY86_H
#define GY86_H

#include <stdint.h>
#include "stm32h7xx_hal.h"
#include "kalman.h"

#define MPU6050_DataRate_8KHz       0   /*!< Sample rate set to 8 kHz */
#define MPU6050_DataRate_4KHz       1   /*!< Sample rate set to 4 kHz */
#define MPU6050_DataRate_2KHz       3   /*!< Sample rate set to 2 kHz */
#define MPU6050_DataRate_1KHz       7   /*!< Sample rate set to 1 kHz */
#define MPU6050_DataRate_500Hz      15  /*!< Sample rate set to 500 Hz */
#define MPU6050_DataRate_250Hz      31  /*!< Sample rate set to 250 Hz */
#define MPU6050_DataRate_125Hz      63  /*!< Sample rate set to 125 Hz */
#define MPU6050_DataRate_100Hz      79  /*!< Sample rate set to 100 Hz */

// Parameters for accelerometer range
typedef enum  {
  MPU6050_Accelerometer_2G = 0x00, /*!< Range is +- 2G */
  MPU6050_Accelerometer_4G = 0x01, /*!< Range is +- 4G */
  MPU6050_Accelerometer_8G = 0x02, /*!< Range is +- 8G */
  MPU6050_Accelerometer_16G = 0x03 /*!< Range is +- 16G */
} MPU6050_Accelerometer;

// Parameters for gyroscope range
typedef enum {
  MPU6050_Gyroscope_250s = 0x00,  /*!< Range is +- 250 degrees/s */
  MPU6050_Gyroscope_500s = 0x01,  /*!< Range is +- 500 degrees/s */
  MPU6050_Gyroscope_1000s = 0x02, /*!< Range is +- 1000 degrees/s */
  MPU6050_Gyroscope_2000s = 0x03  /*!< Range is +- 2000 degrees/s */
} MPU6050_Gyroscope;

typedef struct {
  I2C_HandleTypeDef* i2c;
  uint8_t address; // MPU6050

  float ax;
  float ay;
  float az;
  float temperature;
  float gx;
  float gy;
  float gz;

  float angle_x_offset;
  float angle_y_offset;
  float angle_z_offset;
  float gx_offset;
  float gy_offset;
  float gz_offset;

  float angle_x;
  float angle_y;
  float angle_z;
  float gyro_x;
  float gyro_y;
  float gyro_z;

  // Calculated angles from gyro's values in that order: X, Y, Z
  float gyro_angle[3];
  float acc_angle[3];
  float measures[3];
  long acc_total_vector;
  uint8_t initialized;

  kalman_filter_t kf[6];
  uint8_t rx_buffer[32];
} mpu6050_t;

int MPU6050_init(mpu6050_t *mpu6050, I2C_HandleTypeDef *i2c,
    uint8_t data_rate, MPU6050_Accelerometer accel, MPU6050_Gyroscope gyro);
void MPU6050_set_offset(mpu6050_t *mpu6050,
    float ax_offset, float ay_offset, float az_offset,
    float gx_offset, float gy_offset, float gz_offset);
void MPU6050_update(mpu6050_t *mpu6050);
void MPU6050_parse_6axis(mpu6050_t *mpu6050);

typedef enum {
  OSR_256,
  OSR_512,
  OSR_1024,
  OSR_2048,
  OSR_4096
} OSR;

typedef struct {
  I2C_HandleTypeDef *i2c;

  uint8_t rx_buf[12];
  uint8_t rx_temp[3];
  uint8_t rx_press[3];
  uint8_t tx;
  uint8_t rx;

  uint16_t C[8];
  uint32_t digi_psr_D1;
  uint32_t digi_tem_D2;
  int32_t dT;
  int32_t TEMP;
  int64_t OFF;
  int64_t SENS;
  int32_t P;

  average_filter_t af;
  float fast_pressure;
  float slow_pressure;

  float altitude;

  int OFF2;
  int T2;
  int SENS2;
} ms5611_t;

int MS5611_init(ms5611_t *ms5611, I2C_HandleTypeDef *i2c);

void MS5611_req_temperature(ms5611_t *ms5611, OSR osr);
void MS5611_req_pressure(ms5611_t *ms5611, OSR osr);

void MS5611_read_temperature(ms5611_t *ms5611);
void MS5611_read_pressure(ms5611_t *ms5611);

void MS5611_calc_temperature(ms5611_t *ms5611);
void MS5611_calc_pressure(ms5611_t *ms5611);

void MS5611_calc_altitude(ms5611_t *ms5611);

void MS5611_update(ms5611_t *ms5611);

#endif /* GY86_H */
