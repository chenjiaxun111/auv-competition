#define LOG_TAG "auv_control"
#include <elog.h>
#include <math.h>
#include <pthread.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include "auvControl.h"
#include "../applications/sensor.h"
#include "../connections/rovCommandState.h"
#include "../user/config.h"

typedef struct { uint32_t last_samples; unsigned recovery_frames; int valid, active; } axis_state_t;
static axis_state_t depth_axis, yaw_axis;
static auv_pid_runtime_status_t status_cache;
static pthread_mutex_t status_mutex = PTHREAD_MUTEX_INITIALIZER;
static uint64_t now_ms(void) { struct timespec t; return clock_gettime(CLOCK_MONOTONIC,&t)==0 ? (uint64_t)t.tv_sec*1000U+(uint64_t)t.tv_nsec/1000000U : 0; }
static float clampf_local(float v,float lo,float hi){return v<lo?lo:(v>hi?hi:v);}
static float wrap(float v){while(v>=180)v-=360;while(v< -180)v+=360;return v;}
static int valid(axis_state_t *a,const sensor_health_t *h,uint64_t now){if(!h->last_update_ms||now-h->last_update_ms>AUV_SENSOR_TIMEOUT_MS){a->valid=0;a->recovery_frames=0;a->last_samples=h->sample_count;return 0;}if(h->sample_count!=a->last_samples){a->recovery_frames+=h->sample_count-a->last_samples;a->last_samples=h->sample_count;if(a->recovery_frames>=AUV_SENSOR_RECOVERY_FRAMES)a->valid=1;}return a->valid;}

void auv_control_stop(void){depth_axis.active=yaw_axis.active=0;pid_controller_reset(&pid_controllers_get()->High_Position_Control);pid_controller_reset(&pid_controllers_get()->Yaw_Angle_Control);}

void auv_control(rockerInfo_t *rc,int heartbeat_alive)
{
    depthSensor_t d; jy901_t y; sensor_health_t dh,yh; float relative; uint64_t now=now_ms();
    PID_Controller *dp=&pid_controllers_get()->High_Position_Control,*yp=&pid_controllers_get()->Yaw_Angle_Control;
    sensor_get_depth(&d);sensor_get_jy901(&y);sensor_get_depth_health(&dh);sensor_get_heading_health(&yh);
    if(!heartbeat_alive){auv_control_stop();(void)rov_command_state_take_depth_target(&relative);(void)rov_command_state_take_heading_target(&relative);rc->fx=rc->fy=rc->fz=rc->yaw=0;goto done;}
    depth_axis.valid=valid(&depth_axis,&dh,now);yaw_axis.valid=valid(&yaw_axis,&yh,now);
    if(abs(rc->fz)>AUV_DEPTH_MANUAL_DEADBAND){depth_axis.active=0;pid_controller_reset(dp);(void)rov_command_state_take_depth_target(&relative);}
    else if(!depth_axis.valid){depth_axis.active=0;pid_controller_reset(dp);(void)rov_command_state_take_depth_target(&relative);rc->fz=0;}
    else {if(!depth_axis.active){pid_controller_set_target(dp,d.depth);depth_axis.active=1;}if(rov_command_state_take_depth_target(&relative)>0){float requested=d.depth+relative,target=clampf_local(requested,AUV_DEPTH_TARGET_MIN_M,AUV_DEPTH_TARGET_MAX_M);if(target!=requested)log_w("Depth target %.3fm exceeded range; clamped to %.3fm",requested,target);pid_controller_set_target(dp,target);}rc->fz=(int16_t)(pid_controller_update(dp,d.depth,0)*AUV_DEPTH_OUTPUT_DIRECTION);}
    if(abs(rc->yaw)>AUV_YAW_MANUAL_DEADBAND){yaw_axis.active=0;pid_controller_reset(yp);(void)rov_command_state_take_heading_target(&relative);}
    else if(!yaw_axis.valid){yaw_axis.active=0;pid_controller_reset(yp);(void)rov_command_state_take_heading_target(&relative);rc->yaw=0;}
    else {float feedback=y.yaw*AUV_YAW_FEEDBACK_DIRECTION;if(!yaw_axis.active){pid_controller_set_target(yp,feedback);yaw_axis.active=1;}if(rov_command_state_take_heading_target(&relative)>0)pid_controller_set_target(yp,wrap(feedback+relative));rc->yaw=(int16_t)(pid_controller_update(yp,feedback,1)*AUV_YAW_OUTPUT_DIRECTION);}
done:
    pthread_mutex_lock(&status_mutex);status_cache.timestamp_ms=now;status_cache.depth_active=depth_axis.active;status_cache.depth_sensor_valid=depth_axis.valid;status_cache.yaw_active=yaw_axis.active;status_cache.yaw_sensor_valid=yaw_axis.valid;pid_controller_get_runtime(dp,&status_cache.depth);pid_controller_get_runtime(yp,&status_cache.yaw);pthread_mutex_unlock(&status_mutex);
}
int auv_pid_runtime_status_get(auv_pid_runtime_status_t *s){if(!s)return -1;pthread_mutex_lock(&status_mutex);*s=status_cache;pthread_mutex_unlock(&status_mutex);return 0;}
