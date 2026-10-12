#ifndef FILTER_H
#define FILTER_H

/* 当前项目未调用以下滤波函数，仅保留供后续传感器数据处理使用。 */

#include <stdint.h>

#define FILTER_SAMPLE_COUNT 10U
#define SMOOTH_LENGTH       1U

uint32_t bubble_filter(uint32_t *values);
float kalman_filter(float *value);
float ms5837_smooth(float data);

#endif
