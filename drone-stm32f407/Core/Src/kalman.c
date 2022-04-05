/* 
 * SimpleKalmanFilter - a Kalman Filter implementation for single variable models.
 * Created by Denys Sene, January, 1, 2017.
 * Released under MIT License - see LICENSE file for details.
 */

//#include "Arduino.h"
//#include "SimpleKalmanFilter.h"
//#include "stm32f4xx_hal.h"
#include "kalman.h"
#include "math.h"
#include "String.h"
#include "Stdlib.h"
#include "math.h"
#include "stdio.h"
#include "stdlib.h"
  //float _err_measure, _err_measure1;
  //float _err_estimate, _err_estimate1;
  //float _q, _q1;
  //float _current_estimate, _current_estimate1;
  //float _last_estimate, _last_estimate1;
  //float _kalman_gain, _kalman_gain1;    
//uint8_t convert_char(uint8_t num_char);
void SimpleKalmanFilter_Init(kalman_filter* kalman_unit, float mea_e, float est_e, float q){
  kalman_unit->_err_measure = mea_e;
  kalman_unit->_err_estimate = est_e;
  kalman_unit->_q = q;
}
/*void SimpleKalmanFilter_Init1(float mea_e1, float est_e1, float q1){
  _err_measure1 = mea_e1;
  _err_estimate1 = est_e1;
  _q1 = q1;
}*/
float SimpleKalmanFilter_updateEstimate(kalman_filter* kalman_unit, float mea){
  kalman_unit->_kalman_gain = kalman_unit->_err_estimate/(kalman_unit->_err_estimate + kalman_unit->_err_measure);
  kalman_unit->_current_estimate = kalman_unit->_last_estimate + kalman_unit->_kalman_gain * (mea - kalman_unit->_last_estimate);
  kalman_unit->_err_estimate =  ((float)1.0 - kalman_unit->_kalman_gain)*kalman_unit->_err_estimate + fabs(kalman_unit->_last_estimate-kalman_unit->_current_estimate)*kalman_unit->_q;
  kalman_unit->_last_estimate=kalman_unit->_current_estimate;

  return kalman_unit->_current_estimate;
}
////////////////////////////////////////
float average_filter_update(average_filter* average_unit,float val){
int i=0;
float temp=0;
	while(i<=50){
	average_unit->filter_val[i] = average_unit->filter_val[i+1];
	temp = temp + fabs(average_unit->filter_val[i+1]);
	i++;
	}
	average_unit->result = (average_unit->result + temp + val)/53;
	average_unit->filter_val[i] = val;
	return average_unit->result;
}
/*float SimpleKalmanFilter_updateEstimate1(float mea1){
  _kalman_gain1 = _err_estimate1/(_err_estimate1 + _err_measure1);
  _current_estimate1 = _last_estimate1 + _kalman_gain1 * (mea1 - _last_estimate1);
  _err_estimate1 =  (1.0 - _kalman_gain1)*_err_estimate1 + fabs(_last_estimate1-_current_estimate1)*_q1;
  _last_estimate1=_current_estimate1;

  return _current_estimate1;
}*/
/*void SimpleKalmanFilter::setMeasurementError(float mea_e)
{
  _err_measure=mea_e;
}

void SimpleKalmanFilter::setEstimateError(float est_e)
{
  _err_estimate=est_e;
}

void SimpleKalmanFilter::setProcessNoise(float q)
{
  _q=q;
}

float SimpleKalmanFilter::getKalmanGain() {
  return _kalman_gain;
}  */
///////////////////////////////////////
uint8_t* IntToStr(int32_t num, char x){
	static uint8_t result[20];
	int i=0,i_=0;
	uint32_t temp1, temp2;
	if(num < 0){
		temp2 = -num;
		result[0] = '-';
		i = 1;
		i_ = lengh_num(num);
		} else {
	temp2 = num;
	i_ = lengh_num(num);
	}
	while(i <= i_){
	temp1 = temp2/pow(10, (i_ - i));
	result[i] = convert_char(temp1); 
	//temp2 %= 10;
	temp2 = temp2 - temp1*pow(10, (i_ - i));
	i++;
	}
	//result[i] = '_';
	result[i] = x;
	return result;
}
/* USER CODE END PFP */
uint8_t convert_char(uint8_t num_char){
	//static uint8_t result[1];
	char result = 0;
	switch(num_char){
	case 0: result = '0'; break;
	case 1: result = '1'; break;
  case 2: result = '2'; break;
  case 3: result = '3'; break;		
	case 4: result = '4'; break;
	case 5: result = '5'; break;
  case 6: result = '6'; break;
  case 7: result = '7'; break;
	case 8: result = '8'; break;
  case 9: result = '9'; break;
	}
	return result;
}
/* USER CODE BEGIN 0 */
int lengh_num(int32_t num){
	uint8_t i = 0;
	uint32_t temp1;
	temp1 = abs(num);
	while(temp1 >= 10){
	i++;
	temp1 = temp1/10;
	}
	if(num < 0) {return i+1;} else return i;
}  
