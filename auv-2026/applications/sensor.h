/**
 * @file sensor.h
 * @brief ROV传感器公共数据类型和传感器线程初始化接口。
 *
 * 本头文件只声明其他模块需要访问的深度、电源状态及启动接口。具体传感器
 * 选择、虚拟引脚、线程状态和采集实现均保留在sensor.c内部。
 */

#ifndef __SENSOR_H
#define __SENSOR_H

#include "../drivers/jy901.h"
#include <stdint.h>

/* 深度传感器状态；项目内部深度统一使用米。 */
typedef struct
{
    float depth;       // 相对启动位置的深度，单位m
    float temperature; // 传感器温度，单位℃
} depthSensor_t;

/*
 * 电源状态。
 * TODO：voltage/current当前尚未完成实机换算与标定，详见sensor.c中的
 * adc_thread()注释；标定完成前不得用于安全保护。
 */
typedef struct
{
    float voltage; // 目标单位V；当前实际为未完成分压还原的AIN0电压
    float current; // 目标单位A；当前实际为未完成电流标定的AIN1电压
} powerSource_t;

typedef struct { uint64_t last_update_ms; uint32_t sample_count; } sensor_health_t;

/** 初始化已配置的传感器并创建采集线程。 */
int sensor_thread_init(void);
int sensor_get_depth(depthSensor_t *snapshot);
int sensor_get_power(powerSource_t *snapshot);
int sensor_get_jy901(jy901_t *snapshot);
int sensor_get_depth_health(sensor_health_t *snapshot);
int sensor_get_heading_health(sensor_health_t *snapshot);

#endif
