#include "kalman.h"

#include <string.h>

void kalman_filter_init(kalman_filter_t* filter, float mea_e, float est_e, float q) {
  filter->_err_measure = mea_e;
  filter->_err_estimate = est_e;
  filter->_q = q;
}

float kalman_filter_update(kalman_filter_t* filter, float mea) {
  filter->_kalman_gain = filter->_err_estimate/(filter->_err_estimate + filter->_err_measure);
  filter->_current_estimate = filter->_last_estimate + filter->_kalman_gain * (mea - filter->_last_estimate);
  filter->_err_estimate =  ((float)1.0 - filter->_kalman_gain)*filter->_err_estimate + fabs(filter->_last_estimate-filter->_current_estimate)*filter->_q;
  filter->_last_estimate=filter->_current_estimate;

  return filter->_current_estimate;
}

void average_filter_init(average_filter_t* filter, int width) {
  filter->width = width;
  filter->record_idx = 0;
  filter->sum = 0;
  memset(filter->record, 0, filter->width * sizeof(float));
}

float average_filter_update(average_filter_t* filter, float val) {
  filter->record_idx = (filter->record_idx + 1) % filter->width;
  filter->sum -= filter->record[filter->record_idx];
  filter->sum += val;
  filter->record[filter->record_idx] = val;
  return filter->sum / filter->width;
}
