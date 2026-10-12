#ifndef AUV_CONTROL_H
#define AUV_CONTROL_H
#include <stdint.h>
#include "PID.h"
#include "control.h"
typedef struct { uint64_t timestamp_ms; int depth_active, depth_sensor_valid, yaw_active, yaw_sensor_valid; pid_runtime_t depth, yaw; } auv_pid_runtime_status_t;
void auv_control(rockerInfo_t *rc, int heartbeat_alive);
void auv_control_stop(void);
int auv_pid_runtime_status_get(auv_pid_runtime_status_t *status);
#endif
