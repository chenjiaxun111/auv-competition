/**
 * @file crpcfunc.c
 * @brief 上位机JSON-RPC方法的实现与ROV状态响应。
 *
 * 本文件接收并处理上位机发送的控制、状态查询等RPC请求，将解析后的
 * 控制命令写入共享命令数据，并将ROV状态封装为JSON响应。推进器的实际
 * 混控计算和PWM输出分别由control与pwmDevices模块完成。
 */

#include "../applications/data.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "../applications/pwmDevices.h"
#include "../applications/statusLight.h"
#include "rovRpcData.h"
#include "crpcfunc.h"
#include "../user/config.h"

static cJSON *invalid_params(jrpc_context *ctx, const char *message)
{
    size_t length = strlen(message) + 1U;
    ctx->error_code = JRPC_INVALID_PARAMS;
    ctx->error_message = malloc(length);
    if (ctx->error_message != NULL)
        memcpy(ctx->error_message, message, length);
    return NULL;
}

/**
  * @brief  rov的info信息的rpc函数
  * @param  *params json参数
  * @param  *id  客户端标识
  * @return  info信息的json对象
  */
cJSON *get_status_rpc_handler(jrpc_context *ctx, cJSON *params, cJSON *id)
{
    (void)ctx;
    (void)params;
    (void)id;
    /* 上位机能成功请求状态，说明通信已经建立。 */
    status_light_set(STATUS_LIGHT_NORMAL);
    rov_command_state_note_heartbeat();
    return build_status_json();
} 

/**
  * @brief  运动控制的rpc函数
  * @param  *params json参数
  * @param  *id  客户端标识
  * @return  1：接收成功；0：接收失败
  */
cJSON *move_rpc_handler(jrpc_context *ctx, cJSON *params, cJSON *id)
{
    (void)ctx;
    (void)id;
    if (params == NULL)
        return invalid_params(ctx, "运动参数不能为空");
    cJSON *result = parse_move_params(params);
    if (result == NULL)
        return invalid_params(ctx, "x、y、z、yaw必须是-1到1之间的数字");
    return result;
}

/**
 * @brief 返回最近一次计算并发布的六路推进器目标推力。
 *
 * 返回值是相对1500us停止脉宽的有符号偏移量，不会直接读取PCA9685硬件。
 */
cJSON *get_propeller_power_rpc_handler(jrpc_context *ctx, cJSON *params, cJSON *id)
{
    propellerPower_t power;
    cJSON *result;

    (void)ctx;
    (void)params;
    (void)id;

    if (propeller_power_get(&power) < 0)
        return cJSON_CreateNull();

    result = cJSON_CreateObject();
    if (result == NULL)
        return cJSON_CreateNull();

    cJSON_AddNumberToObject(result, "left_up", power.leftUp);
    cJSON_AddNumberToObject(result, "right_up", power.rightUp);
    cJSON_AddNumberToObject(result, "left_middle", power.leftMiddle);
    cJSON_AddNumberToObject(result, "right_middle", power.rightMiddle);
    cJSON_AddNumberToObject(result, "left_down", power.leftDown);
    cJSON_AddNumberToObject(result, "right_down", power.rightDown);
    return result;
}

/**
  * @brief  机械爪控制的rpc函数
  * @param  *params json参数
  * @param  *id  客户端标识
  * @return  1：接收成功；0：接收失败
  */
cJSON *set_gripper_rpc_handler(jrpc_context *ctx, cJSON *params, cJSON *id)
{
    (void)ctx;
    (void)id;
    if (params == NULL || params->child == NULL ||
        (params->child->type & 0xFF) != cJSON_Number)
        return invalid_params(ctx, "机械爪控制量必须是数字");
    rov_command_state_set_gripper((float)params->child->valuedouble);
    return cJSON_CreateNull();
}

/* AUV相对目标与PID运行状态使用独立RPC，不依赖旧锁定开关。 */
cJSON *set_auv_target_rpc_handler(jrpc_context *ctx,cJSON *params,cJSON *id){double depth=0,heading=0;int hd=0,hh=0;(void)id;
#if VEHICLE_MODEL != VEHICLE_MODEL_AUV
    return invalid_params(ctx,"当前固件为ROV模式，不支持AUV目标");
#else
    if(params){cJSON *d=cJSON_GetObjectItem(params,"relative_depth_meters"),*h=cJSON_GetObjectItem(params,"relative_heading_degrees");if(d){if((d->type&0xff)!=cJSON_Number||!isfinite(d->valuedouble)||d->valuedouble<AUV_RELATIVE_DEPTH_MIN_M||d->valuedouble>AUV_RELATIVE_DEPTH_MAX_M)return invalid_params(ctx,"相对深度必须在-2到2米之间");depth=d->valuedouble;hd=1;}if(h){if((h->type&0xff)!=cJSON_Number||!isfinite(h->valuedouble)||h->valuedouble<AUV_RELATIVE_YAW_MIN_DEG||h->valuedouble>AUV_RELATIVE_YAW_MAX_DEG)return invalid_params(ctx,"相对航向必须在-180到180度之间");heading=h->valuedouble;hh=1;}}
    if(!hd&&!hh)return invalid_params(ctx,"至少提供一个相对目标");rov_command_state_set_auv_target((float)depth,hd,(float)heading,hh);return cJSON_CreateNull();
#endif
}
cJSON *get_pid_runtime_status_rpc_handler(jrpc_context *ctx,cJSON *params,cJSON *id){(void)ctx;(void)params;(void)id;return pid_runtime_status_json_create();}

/**
  * @brief  推进器参数的rpc函数
  * @param  *params json参数
  * @param  *id  客户端标识
  * @return  1：接收成功；0：接收失败
  */
cJSON *set_propeller_parameters_rpc_handler(jrpc_context *ctx, cJSON *params, cJSON *id)
{
    (void)ctx;
    (void)id;
    if (params == NULL) return cJSON_CreateNull();

    if (update_propeller_parameters(params) < 0)
        return invalid_params(ctx, "推进器死区必须在-127到127之间，动力比例必须在0到2之间");
    return cJSON_CreateNull();
}

/**
 * @brief 设置运行中的PID参数
  * @param  *params json参数
  * @param  *id  客户端标识
  * @return  1：接收成功；0：接收失败
  */
cJSON *set_pid_rpc_handler(jrpc_context *ctx, cJSON *params, cJSON *id)
{
    (void)ctx;
    (void)id;

    if (params == NULL)
    {
        return cJSON_CreateNull();
    }

    /* 这里只更新内存；需要永久保存时由save_pid方法写入文件。 */
    if ((cJSON_GetObjectItem(params, "depth_position") != NULL ||
         cJSON_GetObjectItem(params, "depth_speed") != NULL ||
         cJSON_GetObjectItem(params, "yaw_angle") != NULL ||
         cJSON_GetObjectItem(params, "yaw_gyro") != NULL))
    {
        if (parse_pid_params(params, "depth_position") < 0 ||
            parse_pid_params(params, "depth_speed") < 0 ||
            parse_pid_params(params, "yaw_angle") < 0 ||
            parse_pid_params(params, "yaw_gyro") < 0)
            return invalid_params(ctx, "PID参数无效或缺少控制环");
    }

    {
        cJSON *robot_arm = cJSON_GetObjectItem(params, "robot_arm");
        if (robot_arm != NULL && update_robot_arm_parameters(robot_arm) < 0)
            return invalid_params(ctx, "机械臂PWM参数无效：最小值不能大于最大值");
    }

    return cJSON_CreateNull();
}

/**
  * @brief  保存推进器参数的rpc函数
  * @param  *params json参数
  * @param  *id  客户端标识
  * @return  1：接收成功；0：接收失败
  */
cJSON *save_propeller_parameters_rpc_handler(jrpc_context *ctx, cJSON *params, cJSON *id)
{
    (void)ctx;
    (void)params;
    (void)id;
    return cJSON_CreateBool(propeller_parameters_save() == 0);
}

/**
  * @brief 将当前PID参数保存到control_parameters.json
  */
cJSON *save_pid_rpc_handler(jrpc_context *ctx, cJSON *params, cJSON *id)
{
    (void)ctx;
    (void)params;
    (void)id;
    return cJSON_CreateBool(pid_and_pwm_device_parameters_save() == 0);
}

/**
  * @brief  读取推进器参数的rpc函数
  * @param  *params json参数
  * @param  *id  客户端标识
  * @return  1：接收成功；0：接收失败
  */
cJSON *load_parameters_rpc_handler(jrpc_context *ctx, cJSON *params, cJSON *id)
{
    (void)ctx;
    (void)params;
    (void)id;
    if (propeller_parameters_load() < 0 ||
        pid_and_pwm_device_parameters_load() < 0)
        return cJSON_CreateNull();

    return rov_parameters_response_create();
}
