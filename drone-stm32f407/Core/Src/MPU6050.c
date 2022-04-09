#include <math.h>
#include "mpu6050.h"

uint8_t MPU6050_Init(MPU6050_t *mpu6050, I2C_HandleTypeDef *I2Cx,
    uint8_t Gyro_FS, uint8_t Acc_FS, uint8_t DLPF_CFG) {
  mpu6050->I2Cx = I2Cx;

	// Save LSB/Unit for both gyro and acc in order to use them later
	switch(Gyro_FS) {
    case 0: //250dps
      mpu6050->MPU6050_Gyro_LSB = 131.0;
      break;
    case 1: //500dps
      mpu6050->MPU6050_Gyro_LSB = 65.5;
      break;
    case 2: //1000dps
      mpu6050->MPU6050_Gyro_LSB = 32.8;
      break;
    case 3: //2000dps
      mpu6050->MPU6050_Gyro_LSB = 16.4;
      break;
    default:
      mpu6050->MPU6050_Gyro_LSB = 32.8;
      break;
	}

	switch(Acc_FS) {
	case 0: //2g
	  mpu6050->MPU6050_Acc_LSB = 16384.0;
		break;
	case 1: //4g
	  mpu6050->MPU6050_Acc_LSB = 8192.0;
		break;
	case 2: //8g
	  mpu6050->MPU6050_Acc_LSB = 4096.0;
		break;
	case 3: //16g
	  mpu6050->MPU6050_Acc_LSB = 2048.0;
		break;
	default:
	  mpu6050->MPU6050_Acc_LSB = 4096.0;
		break;
	}

	// Read Who am I
	if (HAL_I2C_Mem_Read(mpu6050->I2Cx, MPU6050_ADDR,
	    WHO_AM_I_REG, 1, &mpu6050->rx, 1, 100) != HAL_OK) return 1;
	mpu6050->tx = 0; // Will return this value if code ends here

	// 0x68 will be returned if sensor accessed correctly
	if (mpu6050->rx != 0x68)
	  return 2;

	mpu6050->tx = 0;
  if (HAL_I2C_Mem_Write(mpu6050->I2Cx, MPU6050_ADDR,
      PWR_MGMT_1_REG, 1, &mpu6050->tx, 1, 100) != HAL_OK) return 3;
  HAL_Delay(10);

  mpu6050->tx = 0x00; // Set No Sampling
  if (HAL_I2C_Mem_Write(mpu6050->I2Cx, MPU6050_ADDR,
      SMPLRT_DIV_REG, 1, &mpu6050->tx, 1, 100) != HAL_OK) return 4;
  HAL_Delay(10);

  mpu6050->tx = DLPF_CFG; // Digital Low Pass Filter Setting
  if (HAL_I2C_Mem_Write(mpu6050->I2Cx, MPU6050_ADDR,
      CONFIG_REG, 1, &mpu6050->tx, 1, 100) != HAL_OK) return 5;
  HAL_Delay(10);

  mpu6050->tx = Gyro_FS << 3;
  if (HAL_I2C_Mem_Write(mpu6050->I2Cx, MPU6050_ADDR,
      GYRO_CONFIG_REG, 1, &mpu6050->tx, 1, 100) != HAL_OK) return 6;
  HAL_Delay(10);

  mpu6050->tx = Acc_FS << 3;
  if (HAL_I2C_Mem_Write(mpu6050->I2Cx, MPU6050_ADDR,
      ACCEL_CONFIG_REG, 1, &mpu6050->tx, 1, 100) != HAL_OK) return 7;
  HAL_Delay(10);

	return 0;
}

uint8_t MPU6050_Bypass(MPU6050_t *mpu6050) {
  // Master Disable
  mpu6050->tx = 0b00000000;
	if (HAL_I2C_Mem_Write(mpu6050->I2Cx, MPU6050_ADDR,
	    0x6A, 1, &mpu6050->tx, 1, 100) != HAL_OK) return 1;
	HAL_Delay(10);

	// Bypass Enable
	mpu6050->tx = 0b00000010;
	if (HAL_I2C_Mem_Write(mpu6050->I2Cx, MPU6050_ADDR,
	    0x37, 1, &mpu6050->tx, 1, 100) != HAL_OK) return 2;
	HAL_Delay(10);

	return 0;
}

uint8_t MPU6050_Master(MPU6050_t *mpu6050) {
  // Disable Bypass
  mpu6050->tx = 0x00;
  if (HAL_I2C_Mem_Write(mpu6050->I2Cx, MPU6050_ADDR,
	    0x37, 1, &mpu6050->tx, 1, 100) != HAL_OK) return 1;
	HAL_Delay(10);

	// Master Enable
	mpu6050->tx = 0b00100010;
	if (HAL_I2C_Mem_Write(mpu6050->I2Cx, MPU6050_ADDR,
	    0x6A, 1, &mpu6050->tx, 1, 100) != HAL_OK) return 2;
	HAL_Delay(10);

	// Master Clock to 400kHz
	mpu6050->tx = 0b00001101;
	if (HAL_I2C_Mem_Write(mpu6050->I2Cx, MPU6050_ADDR,
	    0x24, 1, &mpu6050->tx, 1, 100) != HAL_OK) return 3;
	HAL_Delay(10);

	mpu6050->tx = 0x00;
	if (HAL_I2C_Mem_Write(mpu6050->I2Cx, MPU6050_ADDR,
	    PWR_MGMT_1_REG, 1, &mpu6050->tx, 1, 100) != HAL_OK) return 4;
	HAL_Delay(10);

	return 0;
}

uint8_t HMC5883L_Setup(MPU6050_t *mpu6050) {
  // Fill Slave0 DO
  mpu6050->tx = 0b00011000;
  if (HAL_I2C_Mem_Write(mpu6050->I2Cx, HMC5883L_ADDRESS << 1,
      0x00, 1, &mpu6050->tx, 1, 100) != HAL_OK) return 1;
	HAL_Delay(10);

	// Fill Slave0 DO
	mpu6050->tx = 0b00100000;
	if (HAL_I2C_Mem_Write(mpu6050->I2Cx, HMC5883L_ADDRESS << 1,
	    0x01, 1, &mpu6050->tx, 1, 100) != HAL_OK) return 2;
	HAL_Delay(10);

	// Mode: Continuous
	mpu6050->tx = 0x00;
	if (HAL_I2C_Mem_Write(mpu6050->I2Cx, HMC5883L_ADDRESS << 1,
	    0x02, 1, &mpu6050->tx, 1, 100) != HAL_OK) return 3;
	HAL_Delay(10);

	return 0;
}

uint8_t MPU6050_Slave_Read(MPU6050_t *mpu6050) {
  // Access Slave into read mode
	mpu6050->tx = HMC5883L_ADDRESS | 0x80;
	if (HAL_I2C_Mem_Write(mpu6050->I2Cx, MPU6050_ADDR,
	    0x25, 1, &mpu6050->tx, 1, 100) != HAL_OK) return 3;
	HAL_Delay(10);

	// Slave REG for reading to take place
	mpu6050->tx = 0x03;
	if (HAL_I2C_Mem_Write(mpu6050->I2Cx, MPU6050_ADDR,
	    0x26, 1, &mpu6050->tx, 1, 100) != HAL_OK) return 3;
	HAL_Delay(10);

	// Number of data bytes
	mpu6050->tx = 0x80 | 0x06;
	if (HAL_I2C_Mem_Write(mpu6050->I2Cx, MPU6050_ADDR,
	    0x27, 1, &mpu6050->tx, 1, 100) != HAL_OK) return 3;
	HAL_Delay(10);

	return 0;
}

uint8_t MPU6050_DataReady(MPU6050_t *mpu6050) {
	HAL_I2C_Mem_Read(mpu6050->I2Cx, MPU6050_ADDR, INT_STATUS_REG, 1, &mpu6050->rx, 1, 100);
	return mpu6050->rx;
}

//uint8_t MPU6050_Read(MPU6050_t *mpu6050) {
//  uint8_t reg = ACCEL_XOUT_H_REG;
//  while (HAL_I2C_Master_Transmit(mpu6050->I2Cx, (uint16_t) MPU6050_ADDR, &reg, 1, 1) != HAL_OK);
//  while (HAL_I2C_Master_Receive(mpu6050->I2Cx, (uint16_t) MPU6050_ADDR, mpu6050->rx_buf, 20, 1) != HAL_OK);
//  return 0;
//}

uint8_t MPU6050_Read_All(MPU6050_t *mpu6050) {
  if (HAL_I2C_Mem_Read(mpu6050->I2Cx, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1,
      mpu6050->rx_buf, 20, 3) != HAL_OK)
    return 1;

  return 0;
}

uint8_t MPU6050_Read_All_DMA(MPU6050_t *mpu6050) {
  if (HAL_I2C_Mem_Read_DMA(mpu6050->I2Cx, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1,
      mpu6050->rx_buf, 20) != HAL_OK)
    return 1;

  return 0;
}

void MPU6050_Parsing(MPU6050_t *mpu6050) {
  uint8_t *rx_buf = mpu6050->rx_buf;

  mpu6050->Accel_X_RAW = -(rx_buf[0] << 8 | rx_buf[1]);
  mpu6050->Accel_Y_RAW = (rx_buf[2] << 8 | rx_buf[3]);
  mpu6050->Accel_Z_RAW = (rx_buf[4] << 8 | rx_buf[5]);
  // Didn't Save Temp Value
  mpu6050->Gyro_X_RAW = (rx_buf[8] << 8 | rx_buf[9]);
  mpu6050->Gyro_Y_RAW = -(rx_buf[10] << 8 | rx_buf[11]);
  mpu6050->Gyro_Z_RAW = -(rx_buf[12] << 8 | rx_buf[13]);

  mpu6050->Mag_X_RAW = (rx_buf[14] << 8 | rx_buf[15]);
  mpu6050->Mag_Z_RAW = -(rx_buf[16] << 8 | rx_buf[17]);
  mpu6050->Mag_Y_RAW = -(rx_buf[18] << 8 | rx_buf[19]);

	mpu6050->Gyro_X_RAW -= mpu6050->Gyro_X_Offset;
	mpu6050->Gyro_Y_RAW -= mpu6050->Gyro_Y_Offset;
	mpu6050->Gyro_Z_RAW -= mpu6050->Gyro_Z_Offset;

	mpu6050->Mag_X_RAW -= mpu6050->Mag_X_Offset;
	mpu6050->Mag_Y_RAW -= mpu6050->Mag_Y_Offset;
	mpu6050->Mag_Z_RAW -= mpu6050->Mag_Z_Offset;

	mpu6050->Gx = mpu6050->Gyro_X_RAW / mpu6050->MPU6050_Gyro_LSB* D2R;
	mpu6050->Gy = mpu6050->Gyro_Y_RAW / mpu6050->MPU6050_Gyro_LSB* D2R;
	mpu6050->Gz = mpu6050->Gyro_Z_RAW / mpu6050->MPU6050_Gyro_LSB* D2R;
	mpu6050->Ax = mpu6050->Accel_X_RAW / mpu6050->MPU6050_Acc_LSB;
	mpu6050->Ay = mpu6050->Accel_Y_RAW / mpu6050->MPU6050_Acc_LSB;
	mpu6050->Az = mpu6050->Accel_Z_RAW / mpu6050->MPU6050_Acc_LSB;
}

void MPU6050_Parsing_NoOffset(MPU6050_t *mpu6050) {
  uint8_t *rx_buf = mpu6050->rx_buf;

	mpu6050->Accel_X_RAW = -(rx_buf[0] << 8 | rx_buf[1]);
	mpu6050->Accel_Y_RAW = (rx_buf[2] << 8 | rx_buf[3]);
	mpu6050->Accel_Z_RAW = (rx_buf[4] << 8 | rx_buf[5]);
	// Didn't Save Temp Value
	mpu6050->Gyro_X_RAW = (rx_buf[8] << 8 | rx_buf[9]);
	mpu6050->Gyro_Y_RAW = -(rx_buf[10] << 8 | rx_buf[11]);
	mpu6050->Gyro_Z_RAW = -(rx_buf[12] << 8 | rx_buf[13]);

	mpu6050->Mag_X_RAW = (rx_buf[14] << 8 | rx_buf[15]);
	mpu6050->Mag_Z_RAW = -(rx_buf[16] << 8 | rx_buf[17]);
	mpu6050->Mag_Y_RAW = -(rx_buf[18] << 8 | rx_buf[19]);

	mpu6050->Gyro_X_RAW -= mpu6050->Gyro_X_Offset;
	mpu6050->Gyro_Y_RAW -= mpu6050->Gyro_Y_Offset;
	mpu6050->Gyro_Z_RAW -= mpu6050->Gyro_Z_Offset;

	mpu6050->Gx = mpu6050->Gyro_X_RAW / mpu6050->MPU6050_Gyro_LSB* D2R;
	mpu6050->Gy = mpu6050->Gyro_Y_RAW / mpu6050->MPU6050_Gyro_LSB* D2R;
	mpu6050->Gz = mpu6050->Gyro_Z_RAW / mpu6050->MPU6050_Gyro_LSB* D2R;
	mpu6050->Ax = mpu6050->Accel_X_RAW / mpu6050->MPU6050_Acc_LSB;
	mpu6050->Ay = mpu6050->Accel_Y_RAW / mpu6050->MPU6050_Acc_LSB;
	mpu6050->Az = mpu6050->Accel_Z_RAW / mpu6050->MPU6050_Acc_LSB;
}
