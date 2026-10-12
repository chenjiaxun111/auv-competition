#include <stdio.h>
#include <string.h>

#include "rovInfo.h"
#include "../user/config.h"

int rov_info_get_snapshot(rovInfo_t *snapshot)
{
    if (snapshot == NULL)
        return -1;

    memset(snapshot, 0, sizeof(*snapshot));
    snprintf(snapshot->name,
             sizeof(snapshot->name),
             "%s",
             VEHICLE_MODEL == VEHICLE_MODEL_AUV ? "AUV" : "ROV");

    sensor_get_depth(&snapshot->depthSensor);
    sensor_get_power(&snapshot->powerSource);
    sensor_get_jy901(&snapshot->jy901);

    return 0;
}
