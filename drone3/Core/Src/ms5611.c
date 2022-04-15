/*
MS5611.cpp - Class file for the MS5611 Barometric Pressure & Temperature Sensor Arduino Library.
Version: 1.0.0
(c) 2014 Korneliusz Jarzebski
www.jarzebski.pl
This program is free software: you can redistribute it and/or modify
it under the terms of the version 3 GNU General Public License as
published by the Free Software Foundation.
This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.
You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
#include "stm32f4xx_hal.h"
#include "ms5611.h"
#include "math.h"
extern I2C_HandleTypeDef hi2c1;
extern void delay(uint16_t z);

bool begin(MS5611 *dev, ms5611_osr_t osr){
    //Wire.begin();
		while(HAL_I2C_IsDeviceReady(&hi2c1, MS5611_ADDRESS, 3, 100) != HAL_OK);
		while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY);
		setOversampling(dev, osr);
		reset();
		HAL_Delay(100);
    readPROM(dev);
    return true;
}

// Set oversampling value
void setOversampling(MS5611 *dev, ms5611_osr_t osr){
	  dev->uosr = osr;
    switch (osr)
    {
	case MS5611_ULTRA_LOW_POWER:
	    dev->ct = 1;
	    break;
	case MS5611_LOW_POWER:
	    dev->ct = 2;
	    break;
	case MS5611_STANDARD:
	    dev->ct = 3;
	    break;
	case MS5611_HIGH_RES:
	    dev->ct = 4;
	    break;
	case MS5611_ULTRA_HIGH_RES:
	    dev->ct = 5;
	    break;
    }
}

// Get oversampling value
ms5611_osr_t getOversampling(MS5611 dev) {
    return (ms5611_osr_t) dev.uosr;
}

void reset(void){
  while(HAL_I2C_Master_Transmit(&hi2c1, MS5611_ADDRESS, (uint8_t*) MS5611_CMD_RESET, 1, 10) != HAL_OK);
}

void readPROM(MS5611 *dev){
  reset();
  HAL_Delay(3000);
  //read PROM
  uint8_t buf1[16];
  //uint16_t C[8];
  uint8_t CMD_PROM = 0xA0;
  for (int i = 0; i < 8; i++) {
    while (HAL_I2C_Master_Transmit(&hi2c1, MS5611_ADDRESS, (uint8_t *) &CMD_PROM, 1, 100) != HAL_OK);
    while (HAL_I2C_Master_Receive(&hi2c1, MS5611_ADDRESS_READ, &buf1[i*2], 2, 100) != HAL_OK);
    CMD_PROM = CMD_PROM + 2;
  }
  for (uint8_t i = 0; i < 6; i++) {
    dev->fc[i] = (int16_t) buf1[2*i+2] << 8 | (int16_t) buf1[2*i+3];
  }
  crc4(dev->fc);
}
uint32_t RawTemp;
uint32_t readRawTemperature(MS5611 *dev, uint8_t y) {
  uint8_t buf[3];
  uint8_t CMD_ADC_D2 = 0x50 + dev->uosr;
  uint8_t CMD_ADC_READ = 0x00;
  buf[0] = 0x00;buf[1] = 0x00;buf[2] = 0x00;
  if (y==0) {
    while (HAL_I2C_Master_Transmit(&hi2c1, MS5611_ADDRESS, (uint8_t *) &CMD_ADC_D2, 1, 100) != HAL_OK);
  } // send conversion command
  //HAL_Delay(5);
  if(y==3) {
    while (HAL_I2C_Master_Transmit(&hi2c1, MS5611_ADDRESS, (uint8_t *) &CMD_ADC_READ, 1, 100) != HAL_OK); // //send read command
    while (HAL_I2C_Master_Receive(&hi2c1, MS5611_ADDRESS_READ, buf, 3, 100) != HAL_OK);
    uint32_t D1 = (uint32_t)(buf[0] << 16) | (uint32_t)(buf[1] << 8) | buf[2];
    RawTemp = D1;
  }
  return RawTemp;
}
uint32_t RawPres;
uint32_t readRawPressure(MS5611 *dev, uint8_t x){
  uint8_t buf[3];
  uint8_t CMD_ADC_D1 = 0x40 + dev->uosr;
  uint8_t CMD_ADC_READ = 0x00;
  buf[0] = 0x00;buf[1] = 0x00;buf[2] = 0x00;
  if(x==4) {
    while (HAL_I2C_Master_Transmit(&hi2c1, MS5611_ADDRESS, (uint8_t *) &CMD_ADC_D1, 1, 100) != HAL_OK);
  } // send conversion command
  //HAL_Delay(5);
  if (x==7) {
    while (HAL_I2C_Master_Transmit(&hi2c1, MS5611_ADDRESS, (uint8_t *) &CMD_ADC_READ, 1, 100) != HAL_OK); // //send read command
    while (HAL_I2C_Master_Receive(&hi2c1, MS5611_ADDRESS_READ, buf, 3, 100) != HAL_OK);
    uint32_t D2 = (uint32_t)(buf[0] << 16) | (uint32_t)(buf[1] << 8) | buf[2];
    RawPres = D2;
  }
  return RawPres;
}

int32_t readPressure(MS5611* dev, bool compensation, uint8_t x, uint8_t y){
  uint32_t D1 = readRawPressure(dev, x);
  uint32_t D2 = readRawTemperature(dev, y);
  int32_t dT = D2 - (uint32_t)dev->fc[4] * 256;
  int64_t OFF = (int64_t)dev->fc[1] * 65536 + (int64_t)dev->fc[3] * dT / 128;
  int64_t SENS = (int64_t)dev->fc[0] * 32768 + (int64_t)dev->fc[2] * dT / 256;
  if (compensation) {
    int32_t TEMP = 2000 + ((int64_t) dT * dev->fc[5]) / 8388608;
    dev->OFF2 = 0;
    dev->SENS2 = 0;
    if (TEMP < 2000){
      dev->OFF2 = 5 * ((TEMP - 2000) * (TEMP - 2000)) / 2;
      dev->SENS2 = 5 * ((TEMP - 2000) * (TEMP - 2000)) / 4;
    }
    if (TEMP < -1500){
      dev->OFF2 = dev->OFF2 + 7 * ((TEMP + 1500) * (TEMP + 1500));
      dev->SENS2 = dev->SENS2 + 11 * ((TEMP + 1500) * (TEMP + 1500)) / 2;
    }
    OFF = OFF - dev->OFF2;
    SENS = SENS - dev->SENS2;
  }
  uint32_t P = (D1 * SENS / 2097152 - OFF) / 32768;

  return P;
}

double readTemperature(MS5611 *dev, bool compensation, uint8_t y){
  uint32_t D2 = readRawTemperature(dev, y);
  int32_t dT = D2 - (uint32_t)dev->fc[4] * 256;
  int32_t TEMP = 2000 + ((int64_t) dT * dev->fc[5]) / 8388608;
  dev->TEMP2 = 0;
  if (compensation){
    if (TEMP < 2000){dev->TEMP2 = (int32_t) ((dT * dT) / (2 << 3));} //2<<30
  }
  TEMP = TEMP - dev->TEMP2;
  return ((double)TEMP/100);
}

// Calculate altitude from Pressure & Sea level pressure
double getAltitude(double pressure, double seaLevelPressure) {
  return (44330.0 * (1.0 - pow(((double)pressure / (double)seaLevelPressure), 0.1902949)));
  //return ((pow((seaLevelPressure / pressure), 1/5.257) - 1.0)) * ((273.15)) / 0.0065 ;
}

// Calculate sea level from Pressure given on specific altitude
double getSeaLevel(double pressure, double altitude) {
    return ((double)pressure / pow(1.0f - ((double)altitude / 44330.0f), 5.255f));
}

// Read 16-bit from register (oops MSB, LSB)
uint16_t readRegister16(uint8_t reg){
	return 0;
}
// Read 24-bit from register (oops XSB, MSB, LSB)
uint32_t readRegister24(uint8_t reg){
	return 0;
}
//////////////////////////////////////////////////////
unsigned char crc4(unsigned int n_prom[]) {
	int cnt; // simple counter
	unsigned int n_rem; // crc reminder
	unsigned int crc_read; // original value of the crc
	unsigned char n_bit;
	n_rem = 0x00;
	crc_read=n_prom[7]; //save read CRC
	n_prom[7]=(0xFF00 & (n_prom[7])); //CRC byte is replaced by 0
	for (cnt = 0; cnt < 16; cnt++) { // operation is performed on bytes
		// choose LSB or MSB
		if (cnt%2==1) n_rem ^= (unsigned short) ((n_prom[cnt>>1]) & 0x00FF);
		else n_rem ^= (unsigned short) (n_prom[cnt>>1]>>8);
		for (n_bit = 8; n_bit > 0; n_bit--) {
			if (n_rem & (0x8000)) {
				n_rem = (n_rem << 1) ^ 0x3000;
			} else {
				n_rem = (n_rem << 1);
			}
		}
	}
	n_rem= (0x000F & (n_rem >> 12)); // final 4-bit reminder is CRC code
	n_prom[7]=crc_read; // restore the crc_read to its original place
	return (n_rem ^ 0x0);
}


