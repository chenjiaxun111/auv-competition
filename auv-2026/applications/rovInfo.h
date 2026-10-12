#ifndef ROV_INFO_H
#define ROV_INFO_H

#include "sensor.h"

/* 面向RPC和状态上报的ROV传感器快照。 */
typedef struct
{
    char name[10];
    jy901_t jy901;
    powerSource_t powerSource;
    depthSensor_t depthSensor;
} rovInfo_t;

int rov_info_get_snapshot(rovInfo_t *snapshot);

#endif
