#include <math.h>
#include <String.h>
#include <Stdlib.h>
#include "stm32f4xx_hal.h"
////////////////////
typedef struct{
	float _err_measure;
  float _err_estimate;
  float _q;
	float _current_estimate;
  float _last_estimate;
  float _kalman_gain;
}kalman_filter;
////////////////////
typedef struct{
float result;
float filter_val[100];
}average_filter;
////////////////////
float average_filter_update(average_filter* average_unit, float val);
////////////////////
void SimpleKalmanFilter_Init(kalman_filter* kalman_unit, float mea_e, float est_e, float q);
//void SimpleKalmanFilter_Init1(float mea_e1, float est_e1, float q1);
float SimpleKalmanFilter_updateEstimate(kalman_filter* kalman_unit, float mea);
//float SimpleKalmanFilter_updateEstimate1(float mea1);
uint8_t convert_char(uint8_t num_char);
int lengh_num(int32_t num);
uint8_t* IntToStr(int32_t num, char x);
	




