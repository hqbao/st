#ifndef KALMAN_H
#define KALMAN_H

#include <math.h>

#define MAX_AVR_FILTER_WIDTH 100

typedef struct {
	float _err_measure;
  float _err_estimate;
  float _q;
	float _current_estimate;
  float _last_estimate;
  float _kalman_gain;
} kalman_filter_t;

typedef struct {
  int width;
  float sum;
  float record[MAX_AVR_FILTER_WIDTH];
  int record_idx;
} average_filter_t;

void kalman_filter_init(kalman_filter_t* filter, float mea_e, float est_e, float q);

float kalman_filter_update(kalman_filter_t* filter, float mea);

void average_filter_init(average_filter_t* filter, int width);

float average_filter_update(average_filter_t* filter, float val);

#endif
