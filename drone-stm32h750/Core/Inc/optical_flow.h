///////////////////////////////////////
#include "stm32h7xx_hal.h"
#include "stdbool.h"
#define MOUSECAM_CS_LOW      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET)
#define MOUSECAM_CS_HIG      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET)
// ADNS3080 hardware config
#define ADNS3080_PIXELS_X 30
#define ADNS3080_PIXELS_Y 30
#define ADNS3080_CLOCK_SPEED 24000000
// Register Map for the ADNS3080 Optical OpticalFlow Sensor
#define ADNS3080_PRODUCT_ID 0x00
#define ADNS3080_REVISION_ID 0x01
#define ADNS3080_MOTION 0x02
#define ADNS3080_DELTA_X 0x03
#define ADNS3080_DELTA_Y 0x04
#define ADNS3080_SQUAL 0x05
#define ADNS3080_PIXEL_SUM 0x06
#define ADNS3080_MAXIMUM_PIXEL 0x07
#define ADNS3080_CONFIGURATION_BITS 0x0a
#define ADNS3080_EXTENDED_CONFIG 0x0b
#define ADNS3080_DATA_OUT_LOWER 0x0c
#define ADNS3080_DATA_OUT_UPPER 0x0d
#define ADNS3080_SHUTTER_LOWER 0x0e
#define ADNS3080_SHUTTER_UPPER 0x0f
#define ADNS3080_FRAME_PERIOD_LOWER 0x10
#define ADNS3080_FRAME_PERIOD_UPPER 0x11
#define ADNS3080_MOTION_CLEAR 0x12
#define ADNS3080_FRAME_CAPTURE 0x13
#define ADNS3080_SROM_ENABLE 0x14
#define ADNS3080_FRAME_PERIOD_MAX_BOUND_LOWER 0x19
#define ADNS3080_FRAME_PERIOD_MAX_BOUND_UPPER 0x1a
#define ADNS3080_FRAME_PERIOD_MIN_BOUND_LOWER 0x1b
#define ADNS3080_FRAME_PERIOD_MIN_BOUND_UPPER 0x1c
#define ADNS3080_SHUTTER_MAX_BOUND_LOWER 0x1e
#define ADNS3080_SHUTTER_MAX_BOUND_UPPER 0x1e
#define ADNS3080_SROM_ID 0x1f
#define ADNS3080_OBSERVATION 0x3d
#define ADNS3080_INVERSE_PRODUCT_ID 0x3f
#define ADNS3080_PIXEL_BURST 0x40
#define ADNS3080_MOTION_BURST 0x50
#define ADNS3080_SROM_LOAD 0x60 
// Extended Configuration bits
#define ADNS3080_SERIALNPU_OFF 0x02
//////////////////////////////
typedef struct{
    int8_t motion;
    int8_t dx;
    int8_t dy;
		int16_t x_pos;
    int16_t y_pos;
    char surface_qual;
    char Shutter_Upper_;
    char Shutter_Lower_;
    char Maximum_px;
    } ADNS3080_str;      //structure for storing the values that are returned by the burst read procedure (refer to the datasheet)

uint8_t O_F_Read(uint8_t reg);
void O_F_Write(uint8_t reg, uint8_t val);
void burst_read(ADNS3080_str* temp);
void init_sensor(void);
void Write_SROM(void);		
//////////////////////////////////////////	


