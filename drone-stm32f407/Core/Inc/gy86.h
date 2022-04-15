#ifndef GY86_H
#define GY86_H

#include <stdint.h>
#include "stm32f4xx_hal.h"

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
} mpu6050_t;

typedef enum {
  OSR_ULTRA_HIGH = 12,     // 10 millis
  OSR_HIGH = 11,           //  5 millis
  OSR_STANDARD = 10,       //  3 millis
  OSR_LOW = 9,             //  2 millis
  OSR_ULTRA_LOW = 8        //  1 millis     Default = backwards compatible
} osr;

typedef struct {
  I2C_HandleTypeDef* i2c;

  uint8_t address;
  uint8_t sampling_rate;
  int32_t temperature;
  int32_t pressure;
  float pressure_offset;
  float temperature_offset;
  int result;
  float C[7];
  uint32_t device_ID;
  uint8_t compensation;
} ms5611_t;

int MPU6050_init(mpu6050_t *mpu6050, I2C_HandleTypeDef *i2c,
    uint8_t data_rate, MPU6050_Accelerometer accel, MPU6050_Gyroscope gyro);
void MPU6050_update(mpu6050_t *mpu6050);
void MPU6050_i2c_mem_read_cb_handler(mpu6050_t *mpu6050);

int MS5611_init(ms5611_t *ms5611, I2C_HandleTypeDef *i2c, uint8_t device_addr);
void MS5611_set_oversampling(ms5611_t *ms5611, osr sampling_rate);
int MS5611_read(ms5611_t *ms5611);
float MS5611_getTemperature(ms5611_t *ms5611);
float MS5611_getPressure(ms5611_t *ms5611);

#endif /* GY86_H */
