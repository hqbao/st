#include "gy86.h"

#define MS5611_NOT_READ           -999

// Datasheet page 10
#define MS5611_CMD_READ_ADC       0x00
#define MS5611_CMD_READ_PROM      0xA0
#define MS5611_CMD_RESET          0x1E
#define MS5611_CMD_CONVERT_D1     0x40
#define MS5611_CMD_CONVERT_D2     0x50

void _command(ms5611_t *ms5611, uint8_t command) {
  ms5611->result = MS5611_NOT_READ;
  if (HAL_I2C_Master_Transmit(ms5611->i2c, ms5611->address << 1, &command, 1, 100) == HAL_OK) {
    ms5611->result = 0;
    return;
  }
}

void _convert(ms5611_t *ms5611, uint8_t addr, uint8_t bits) {
  uint8_t index = bits;
  if (index < 8) index = 8;
  else if (index > 12) index = 12;
  index -= 8;
  uint8_t offset = index * 2;
  _command(ms5611, addr + offset);

  // Values from page 3 datasheet - MAX column (rounded up)
  static const uint16_t del[5] = {600, 1200, 2300, 4600, 9100};

  // // While loop prevents blocking RTOS
  // uint16_t wait_time = del[index];
  // uint32_t start = micros();
  // while (micros() - start < wait_time) {
  //   delayMicroseconds(10);
  // }
  HAL_Delay(del[index]/1000);
}

uint32_t _read_ADC(ms5611_t *ms5611) {
  _command(ms5611, MS5611_CMD_READ_ADC);
  if (ms5611->result == 0) {
    uint8_t buf[3] = {0};
    if (HAL_I2C_Master_Receive(ms5611->i2c, (ms5611->address << 1) | 0x01, buf, 3, 100) != HAL_OK)
      ms5611->result = MS5611_NOT_READ;

    uint32_t raw_temperature = (uint32_t)(buf[0] << 16) | (uint32_t)(buf[1] << 8) | buf[2];
    return raw_temperature;
  }

  return 0;
}

uint16_t _read_PROM(ms5611_t *ms5611, uint8_t reg) {
  // Last EEPROM register is CRC - Page 13 datasheet.
  uint8_t prom_CRC_reg = 7;
  if (reg > prom_CRC_reg) return 0;

  uint8_t offset = reg * 2;
  _command(ms5611, MS5611_CMD_READ_PROM + offset);
  if (ms5611->result == 0) {
    uint8_t buf[2] = {0};
    if (HAL_I2C_Master_Receive(ms5611->i2c, ms5611->address << 1, buf, 2, 100) != HAL_OK)
      ms5611->result = MS5611_NOT_READ;

    uint16_t value = (int16_t) buf[0] << 8 | (int16_t) buf[1];
    return value;
  }

  return 0;
}

uint8_t _reset(ms5611_t *ms5611) {
  _command(ms5611, MS5611_CMD_RESET);

  // // While loop prevents blocking RTOS
  // uint32_t start = micros();
  // while (micros() - start < 2800) {
  //   delayMicroseconds(10);
  // }
  HAL_Delay(2800);

  // Constants that were multiplied in read()
  // Do this once and you save CPU cycles
  ms5611->C[0] = 1;
  ms5611->C[1] = 32768L;          // SENSt1   = C[1] * 2^15
  ms5611->C[2] = 65536L;          // OFFt1    = C[2] * 2^16
  ms5611->C[3] = 3.90625E-3;      // TCS      = C[3] / 2^6
  ms5611->C[4] = 7.8125E-3;       // TCO      = C[4] / 2^7
  ms5611->C[5] = 256;             // Tref     = C[5] * 2^8
  ms5611->C[6] = 1.1920928955E-7; // TEMPSENS = C[6] / 2^23

  // Read factory calibrations from EEPROM.
  uint8_t ROM_OK = 1;
  for (uint8_t reg = 0; reg < 7; reg++) {
    // Used indices match datasheet.
    // C[0] == manufacturer - read but not used;
    // C[7] == CRC - skipped.
    uint16_t tmp = _read_PROM(ms5611, reg);
    ms5611->C[reg] *= tmp;
    // ms5611->device_ID is a simple SHIFT XOR merge of PROM data
    ms5611->device_ID <<= 4;
    ms5611->device_ID ^= tmp;

    if (reg > 0) {
      ROM_OK = ROM_OK && (tmp != 0);
    }
  }

  return ROM_OK;
}

int MS5611_init(ms5611_t *ms5611, I2C_HandleTypeDef *i2c, uint8_t device_addr) {
  ms5611->i2c = i2c;
  ms5611->address            = device_addr;
  ms5611->sampling_rate      = OSR_STANDARD;
  ms5611->temperature        = MS5611_NOT_READ;
  ms5611->pressure           = MS5611_NOT_READ;
  ms5611->result             = MS5611_NOT_READ;
  ms5611->device_ID          = 0;
  ms5611->pressure_offset    = 0;
  ms5611->temperature_offset = 0;
  ms5611->compensation       = 1;

  if (ms5611->address < 0x76 || ms5611->address > 0x77) return 1;

  if (HAL_I2C_IsDeviceReady(ms5611->i2c, ms5611->address << 1, 100, 1000) != HAL_OK) return 2;

  if (!_reset(ms5611))
    return 3;

  return 0;
}

void MS5611_set_oversampling(ms5611_t *ms5611, osr sampling_rate) {
  ms5611->sampling_rate = (uint8_t) sampling_rate;
}

int MS5611_read(ms5611_t *ms5611) {
  // VARIABLES NAMES BASED ON DATASHEET
  // ALL MAGIC NUMBERS ARE FROM DATASHEET

  _convert(ms5611, MS5611_CMD_CONVERT_D1, ms5611->sampling_rate);
  if (ms5611->result) return 1;
  // NOTE: D1 and D2 seem reserved in MBED (NANO BLE)
  uint32_t _D1 = _read_ADC(ms5611);
  if (ms5611->result) return 2;

  _convert(ms5611, MS5611_CMD_CONVERT_D2, ms5611->sampling_rate);
  if (ms5611->result) return 3;
  uint32_t _D2 = _read_ADC(ms5611);
  if (ms5611->result) return 4;

  // TEMP & PRESS MATH - PAGE 7/20
  float dT = _D2 - ms5611->C[5];
  ms5611->temperature = 2000 + dT * ms5611->C[6];

  float offset =  ms5611->C[2] + dT * ms5611->C[4];
  float sens = ms5611->C[1] + dT * ms5611->C[3];

  if (ms5611->compensation == 1) {
    // SECOND ORDER COMPENSATION - PAGE 8/20
    // COMMENT OUT < 2000 CORRECTION IF NOT NEEDED
    // NOTE TEMPERATURE IS IN 0.01 C
    if (ms5611->temperature < 2000) {
      float T2 = dT * dT * 4.6566128731E-10;
      float t = (ms5611->temperature - 2000) * (ms5611->temperature - 2000);
      float offset2 = 2.5 * t;
      float sens2 = 1.25 * t;
      // COMMENT OUT < -1500 CORRECTION IF NOT NEEDED
      if (ms5611->temperature < -1500) {
        t = (ms5611->temperature + 1500) * (ms5611->temperature + 1500);
        offset2 += 7 * t;
        sens2 += 5.5 * t;
      }
      ms5611->temperature -= T2;
      offset -= offset2;
      sens -= sens2;
    }
    // END SECOND ORDER COMPENSATION
  }

  ms5611->pressure = (_D1 * sens * 4.76837158205E-7 - offset) * 3.051757813E-5;

  return 0;
}

float MS5611_getTemperature(ms5611_t *ms5611) {
  if (ms5611->temperature_offset == 0)
    return ms5611->temperature * 0.01;

  return ms5611->temperature * 0.01 + ms5611->temperature_offset;
};

float MS5611_getPressure(ms5611_t *ms5611) {
  if (ms5611->pressure_offset == 0)
    return ms5611->pressure * 0.01;

  return ms5611->pressure * 0.01 + ms5611->pressure_offset;
};
