/**
 * @file filter.c
 * @brief 预留的中值、卡尔曼和深度平滑滤波函数。
 * @note 当前项目未调用本文件中的函数，接入传感器前需结合采样频率调参。
 */

#include <math.h>
#include <stddef.h>
#include <stdint.h>

#include "filter.h"

uint32_t bubble_filter(uint32_t *values)
{
    uint32_t i;
    uint32_t j;

    if (values == NULL)
        return 0;

    for (j = 0; j + 1 < FILTER_SAMPLE_COUNT; j++)
    {
        int swapped = 0;
        for (i = 0; i + 1 < FILTER_SAMPLE_COUNT - j; i++)
        {
            if (values[i] > values[i + 1])
            {
                uint32_t temporary = values[i];
                values[i] = values[i + 1];
                values[i + 1] = temporary;
                swapped = 1;
            }
        }
        if (!swapped)
            break;
    }

    return (values[4] + values[5] + values[6]) / 3U;
}

float kalman_filter(float *value)
{
    static float previous_data;
    static float covariance = 10.0f;
    const float process_noise = 0.0001f;
    const float measurement_noise = 0.001f;
    float gain;

    if (value == NULL || !isfinite(*value))
        return previous_data;

    covariance += process_noise;
    gain = covariance / (covariance + measurement_noise);
    *value = previous_data + gain * (*value - previous_data);
    covariance = (1.0f - gain) * covariance;
    previous_data = *value;
    return *value;
}

float ms5837_smooth(float data)
{
    static float previous_data;
    static float depth_last;
    static uint8_t initialized;

    if (!isfinite(data))
        return initialized ? depth_last : 0.0f;

    if (!initialized)
    {
        previous_data = data;
        depth_last = data;
        initialized = 1;
        return data;
    }

    if (fabsf(depth_last - data) > 1.0f)
        return depth_last;

    depth_last = (previous_data + data) / (SMOOTH_LENGTH + 1.0f);
    previous_data = data;
    return depth_last;
}
