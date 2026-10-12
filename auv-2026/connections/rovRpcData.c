/**
 * @file rovRpcData.c
 * @brief RPC请求参数解析、运行状态更新和响应JSON生成。
 */

#define LOG_TAG "rpc"

#include <stdlib.h>
#include <elog.h>
#include <string.h>
#include <math.h>

#include "rovRpcData.h"
#include "../applications/rovInfo.h"
#include "../applications/pwmDevices.h"
#include "../control/PID.h"
#include "../control/auvControl.h"
#include "../control/auvControl.h"


static void parse_propeller_params(
    cJSON *params,
    propeller_runtime_config_t *component,
    const char *str);

static int validate_propeller_component(cJSON *params, const char *name)
{
    const char *integer_names[] = {"deadzone_upper", "deadzone_lower"};
    const char *power_names[] = {"power_positive", "power_negative"};
    const char *bool_names[] = {"reversed", "enabled"};
    cJSON *component = cJSON_GetObjectItem(params, name);
    size_t i;

    if (component == NULL) return 0; /* 允许RPC只更新一部分推进器。 */
    if ((component->type & 0xFF) != cJSON_Object) return -1;
    for (i = 0; i < 2; ++i) {
        cJSON *item = cJSON_GetObjectItem(component, integer_names[i]);
        if (item != NULL && ((item->type & 0xFF) != cJSON_Number ||
            !isfinite(item->valuedouble) || item->valuedouble != item->valueint ||
            item->valueint < -127 || item->valueint > 127)) return -1;
    }
    for (i = 0; i < 2; ++i) {
        cJSON *item = cJSON_GetObjectItem(component, power_names[i]);
        if (item != NULL && ((item->type & 0xFF) != cJSON_Number ||
            !isfinite(item->valuedouble) || item->valuedouble < 0.0 ||
            item->valuedouble > 2.0)) return -1;
    }
    for (i = 0; i < 2; ++i) {
        cJSON *item = cJSON_GetObjectItem(component, bool_names[i]);
        int type;
        if (item == NULL) continue;
        type = item->type & 0xFF;
        if (type != cJSON_True && type != cJSON_False &&
            (type != cJSON_Number ||
             (item->valueint != 0 && item->valueint != 1))) return -1;
    }
    return 0;
}

/**
 * @brief 从JSON对象读取数值字段。
 * @return 成功返回1，字段缺失、类型错误或参数无效返回0。
 */
static int get_number(const cJSON *object, const char *key, double *value)
{
    cJSON *item;

    if (object == NULL || key == NULL || value == NULL)
        return 0;

    item = cJSON_GetObjectItem((cJSON *)object, key);
    if (item == NULL || (item->type & 0xFF) != cJSON_Number)
        return 0;

    *value = item->valuedouble;
    return 1;
}

/**
 * @brief 从JSON对象读取整数或布尔字段。
 * @return 成功返回1，字段缺失、类型错误或参数无效返回0。
 */
static int get_integer_or_bool(const cJSON *object, const char *key, int *value)
{
    cJSON *item;
    int type;

    if (object == NULL || key == NULL || value == NULL)
        return 0;

    item = cJSON_GetObjectItem((cJSON *)object, key);
    if (item == NULL)
        return 0;

    type = item->type & 0xFF;
    if (type != cJSON_Number && type != cJSON_True && type != cJSON_False)
        return 0;

    *value = item->valueint;
    return 1;
}

/**
 * @brief 创建上位机状态响应，包含水温、航向角和深度。
 * @return 新建的JSON对象，创建失败返回NULL；调用者负责释放。
 */
cJSON *build_status_json(void)
{
    rovInfo_t rov_info;
    cJSON *status = cJSON_CreateObject();

    if (status == NULL)
        return NULL;

    rov_info_get_snapshot(&rov_info);
    cJSON_AddNumberToObject(status, "temperature_celsius",
                           rov_info.depthSensor.temperature);
    cJSON_AddNumberToObject(status, "heading_degrees", rov_info.jy901.yaw);
    cJSON_AddNumberToObject(status, "depth_meters", rov_info.depthSensor.depth);
    return status;
}

/**
 * @brief 解析上位机运动指令并更新RAM中的摇杆状态。
 * @param params 包含x、y、z以及yaw的JSON对象。
 * @return 新建的JSON null对象，供JSON-RPC作为空结果返回。
 */
cJSON *parse_move_params(cJSON *params)
{
    double x, y, z, yaw;
    static int previous_valid;
    static int16_t previous_x;
    static int16_t previous_y;
    static int16_t previous_z;
    static int16_t previous_yaw;
    int16_t command_x;
    int16_t command_y;
    int16_t command_z;
    int16_t command_yaw;

    if (!get_number(params, "x", &x) || !get_number(params, "y", &y) ||
        !get_number(params, "z", &z) || !get_number(params, "yaw", &yaw) ||
        !isfinite(x) || !isfinite(y) || !isfinite(z) || !isfinite(yaw) ||
        x < -1.0 || x > 1.0 || y < -1.0 || y > 1.0 ||
        z < -1.0 || z > 1.0 || yaw < -1.0 || yaw > 1.0)
        return NULL;

    command_x = (int16_t)(x * 127.0);
    command_y = (int16_t)(y * 127.0);
    command_z = (int16_t)(z * 127.0);
    command_yaw = (int16_t)(yaw * 127.0);

    rov_command_state_set_move(command_x, command_y, command_z, command_yaw);
    if (!previous_valid || command_x != previous_x || command_y != previous_y ||
        command_z != previous_z || command_yaw != previous_yaw)
    {
        log_i("MOVE x(sway)=%+.3f y(surge)=%+.3f z(heave)=%+.3f yaw=%+.3f",
              x, y, z, yaw);
        previous_x = command_x;
        previous_y = command_y;
        previous_z = command_z;
        previous_yaw = command_yaw;
        previous_valid = 1;
    }
    return cJSON_CreateNull();
}

/**
 * @brief 从推进器参数对象解析一台推进器的配置。
 * @param params 包含六路推进器子对象的JSON对象。
 * @param component 接收配置的目标推进器。
 * @param str 目标推进器在JSON中的键名。
 */
static void parse_propeller_params(cJSON *params, propeller_runtime_config_t *component, const char *str)
{
    cJSON *cjson_temp;
    double number;
    int integer;

    if (params == NULL || component == NULL || str == NULL)
        return;

    cjson_temp = cJSON_GetObjectItem(params, str);
    if (cjson_temp == NULL || (cjson_temp->type & 0xFF) != cJSON_Object)
        return;

    if (get_integer_or_bool(cjson_temp, "deadzone_upper", &integer))
        component->deadzone_upper = integer;
    if (get_integer_or_bool(cjson_temp, "deadzone_lower", &integer))
        component->deadzone_lower = integer;
    if (get_integer_or_bool(cjson_temp, "enabled", &integer))
        component->enabled = integer != 0;
    if (get_integer_or_bool(cjson_temp, "reversed", &integer))
        component->direction_multiplier = integer != 0 ? -1 : 1;
    if (get_number(cjson_temp, "power_negative", &number))
        component->power_negative = number;
    if (get_number(cjson_temp, "power_positive", &number))
        component->power_positive = number;
}

/**
 * @brief 线程安全地解析并更新六路推进器运行参数。
 * @param params propeller_parameters对象，不取得其内存所有权。
 */
int update_propeller_parameters(cJSON *params)
{
    rov_command_state_t snapshot;
    const char *names[] = {"back_left", "back_right", "center_left",
                           "center_right", "front_left", "front_right"};
    size_t i;

    if (params == NULL)
        return -1;
    for (i = 0; i < sizeof(names) / sizeof(names[0]); ++i)
        if (validate_propeller_component(params, names[i]) < 0)
            return -1;
    if (rov_command_state_get_snapshot(&snapshot) < 0)
        return -1;
    parse_propeller_params(params, &snapshot.propellers.back_left, "back_left");
    parse_propeller_params(params, &snapshot.propellers.back_right, "back_right");
    parse_propeller_params(params, &snapshot.propellers.center_left, "center_left");
    parse_propeller_params(params, &snapshot.propellers.center_right, "center_right");
    parse_propeller_params(params, &snapshot.propellers.front_left, "front_left");
    parse_propeller_params(params, &snapshot.propellers.front_right, "front_right");
    rov_command_state_update_propellers(&snapshot.propellers);
    return 0;
}

int update_robot_arm_parameters(cJSON *params)
{
    pwm_device_t arm;
    double min_pwm;
    double max_pwm;
    double speed;

    if (params == NULL || (params->type & 0xFF) != cJSON_Object ||
        !get_number(params, "min_pwm", &min_pwm) ||
        !get_number(params, "max_pwm", &max_pwm) ||
        !get_number(params, "speed", &speed) ||
        !isfinite(min_pwm) || !isfinite(max_pwm) || !isfinite(speed) ||
        min_pwm < 0.0 || max_pwm > 65535.0 || speed < 0.0 ||
        speed > 65535.0 || min_pwm > max_pwm ||
        min_pwm != (uint16_t)min_pwm || max_pwm != (uint16_t)max_pwm ||
        speed != (uint16_t)speed || robot_arm_get(&arm) < 0)
        return -1;

    arm.nMax = (uint16_t)min_pwm;
    arm.pMax = (uint16_t)max_pwm;
    arm.speed = (uint16_t)speed;
    return robot_arm_set_config(&arm);
}

/**
 * @brief 将一台推进器的运行配置加入JSON对象。
 * @param propellers 接收推进器子对象的父JSON对象。
 * @param name 推进器键名。
 * @param config 推进器配置来源。
 */
static void add_propeller_json(cJSON *propellers, const char *name,
                               const propeller_runtime_config_t *config)
{
    cJSON *propeller = cJSON_CreateObject();

    if (propeller == NULL)
        return;

    cJSON_AddNumberToObject(propeller, "deadzone_lower", config->deadzone_lower);
    cJSON_AddNumberToObject(propeller, "deadzone_upper", config->deadzone_upper);
    cJSON_AddNumberToObject(propeller, "power_positive", config->power_positive);
    cJSON_AddNumberToObject(propeller, "power_negative", config->power_negative);
    if (config->direction_multiplier < 0)
        cJSON_AddTrueToObject(propeller, "reversed");
    else
        cJSON_AddFalseToObject(propeller, "reversed");
    if (config->enabled != 0)
        cJSON_AddTrueToObject(propeller, "enabled");
    else
        cJSON_AddFalseToObject(propeller, "enabled");
    cJSON_AddItemToObject(propellers, name, propeller);
}

/**
 * @brief 根据RAM中的六路推进器配置创建可持久化JSON。
 * @return 新建的根JSON对象，创建失败返回NULL；调用者负责释放。
 */
cJSON *propeller_parameters_json_create(void)
{
    rov_command_state_t snapshot;
    cJSON *root = cJSON_CreateObject();
    cJSON *propellers = cJSON_CreateObject();

    if (root == NULL || propellers == NULL || rov_command_state_get_snapshot(&snapshot) < 0)
    {
        cJSON_Delete(root);
        cJSON_Delete(propellers);
        return NULL;
    }

    add_propeller_json(propellers, "front_left", &snapshot.propellers.front_left);
    add_propeller_json(propellers, "front_right", &snapshot.propellers.front_right);
    add_propeller_json(propellers, "center_left", &snapshot.propellers.center_left);
    add_propeller_json(propellers, "center_right", &snapshot.propellers.center_right);
    add_propeller_json(propellers, "back_left", &snapshot.propellers.back_left);
    add_propeller_json(propellers, "back_right", &snapshot.propellers.back_right);
    cJSON_AddItemToObject(root, "propeller_parameters", propellers);
    return root;
}

/**
 * @brief 从上位机请求中解析并更新一组PID增益。
 * @param params 包含PID分组的JSON对象。
 * @param str PID分组名：depth_position、depth_speed、yaw_angle、yaw_gyro。
 */
#if 0
int parse_pid_params(cJSON *params, const char *str)
{
    cJSON *cjson_temp;
    PID_Controler *controller;
    pid_values_t pid;
    double value;

    if (params == NULL || str == NULL)
        return -1;

    if (!strcmp(str, "yaw_angle"))
        controller = &pid_controllers_get()->Yaw_Angle_Control;
    else if (!strcmp(str, "depth_position"))
        controller = &pid_controllers_get()->High_Position_Control;
    else if (!strcmp(str, "depth_speed"))
        controller = &pid_controllers_get()->High_Speed_Control;
    else if (!strcmp(str, "yaw_gyro"))
        controller = &pid_controllers_get()->Yaw_Gyro_Control;
    else
    {
        log_e("Unknown PID group: %s", str);
        return -1;
    }

    cjson_temp = cJSON_GetObjectItem(params, str);
    if (cjson_temp == NULL || (cjson_temp->type & 0xFF) != cJSON_Object)
        return -1;

    cJSON *p_item = cJSON_GetObjectItem(cjson_temp, "p");
    cJSON *i_item = cJSON_GetObjectItem(cjson_temp, "i");
    cJSON *d_item = cJSON_GetObjectItem(cjson_temp, "d");
    cJSON *output_item = cJSON_GetObjectItem(cjson_temp, "output_limit");
    cJSON *integral_item = cJSON_GetObjectItem(cjson_temp, "integral_limit");
    cJSON *separation_item = cJSON_GetObjectItem(cjson_temp, "integral_separation_error");
    if ((p_item != NULL && (p_item->type & 0xFF) != cJSON_Number) ||
        (i_item != NULL && (i_item->type & 0xFF) != cJSON_Number) ||
        (d_item != NULL && (d_item->type & 0xFF) != cJSON_Number) ||
        (output_item != NULL && (output_item->type & 0xFF) != cJSON_Number) ||
        (integral_item != NULL && (integral_item->type & 0xFF) != cJSON_Number) ||
        (separation_item != NULL && (separation_item->type & 0xFF) != cJSON_Number))
        return -1;

    {
        float kp, ki, kd;
        if (pid_controller_get_gains(controller, &kp, &ki, &kd) < 0)
            return -1;
        pid.p = kp; pid.i = ki; pid.d = kd;
    }
    if (get_number(cjson_temp, "p", &value)) pid.p = value;
    if (get_number(cjson_temp, "i", &value)) pid.i = value;
    if (get_number(cjson_temp, "d", &value)) pid.d = value;

    if (!isfinite(pid.p) || !isfinite(pid.i) || !isfinite(pid.d) ||
        fabs(pid.p) > 10000.0 || fabs(pid.i) > 10000.0 ||
        fabs(pid.d) > 10000.0)
        return -1;

    if (get_number(cjson_temp, "output_limit", &value)) {
        if (!isfinite(value) || value < 0.0 || value > 10000.0) return -1;
        controller->Control_OutPut_Limit = (float)value;
    }
    if (get_number(cjson_temp, "integral_limit", &value)) {
        if (!isfinite(value) || value < 0.0 || value > 10000.0) return -1;
        controller->Integrate_Max = (float)value;
    }
    if (get_number(cjson_temp, "integral_separation_error", &value)) {
        if (!isfinite(value) || value < 0.0 || value > 10000.0) return -1;
        controller->Integrate_Separation_Err = (float)value;
    }
    {
        int enabled;
        if (get_integer_or_bool(cjson_temp, "integral_limit_enabled", &enabled)) {
            if (enabled != 0 && enabled != 1) return -1;
            controller->Integrate_Limit_Flag = (uint8_t)enabled;
        }
        if (get_integer_or_bool(cjson_temp, "integral_separation_enabled", &enabled)) {
            if (enabled != 0 && enabled != 1) return -1;
            controller->Integrate_Separation_Flag = (uint8_t)enabled;
        }
    }

    return pid_controller_set_gains(controller, (float)pid.p,
                                    (float)pid.i, (float)pid.d);
}

/**
 * @brief 创建推进器参数及定深、定向PID参数的RPC响应。
 * @return 新建的JSON对象，创建失败时尽量返回空对象；调用者负责释放。
 */
static cJSON *pid_parameters_json_create(const PID_Controler *controller)
{
    cJSON *pid = cJSON_CreateObject();
    float p;
    float i;
    float d;

    if (pid == NULL || controller == NULL ||
        pid_controller_get_gains(controller, &p, &i, &d) < 0)
    {
        cJSON_Delete(pid);
        return NULL;
    }

    cJSON_AddNumberToObject(pid, "p", p);
    cJSON_AddNumberToObject(pid, "i", i);
    cJSON_AddNumberToObject(pid, "d", d);
    cJSON_AddNumberToObject(pid, "output_limit", controller->Control_OutPut_Limit);
    cJSON_AddNumberToObject(pid, "integral_limit", controller->Integrate_Max);
    if (controller->Integrate_Limit_Flag != 0)
        cJSON_AddTrueToObject(pid, "integral_limit_enabled");
    else
        cJSON_AddFalseToObject(pid, "integral_limit_enabled");
    cJSON_AddNumberToObject(pid, "integral_separation_error",
                            controller->Integrate_Separation_Err);
    if (controller->Integrate_Separation_Flag != 0)
        cJSON_AddTrueToObject(pid, "integral_separation_enabled");
    else
        cJSON_AddFalseToObject(pid, "integral_separation_enabled");
    return pid;
}
#endif

static PID_Controller *pid_by_name(const char *name)
{
    AllController *all = pid_controllers_get();
    if (!strcmp(name,"depth_position")) return &all->High_Position_Control;
    if (!strcmp(name,"depth_speed")) return &all->High_Speed_Control;
    if (!strcmp(name,"yaw_angle")) return &all->Yaw_Angle_Control;
    if (!strcmp(name,"yaw_gyro")) return &all->Yaw_Gyro_Control;
    return NULL;
}

static int json_uint(cJSON *o,const char *key,uint16_t *v,uint16_t max)
{
    cJSON *i=cJSON_GetObjectItem(o,key); if(!i)return 0;
    if((i->type&0xff)!=cJSON_Number||!isfinite(i->valuedouble)||i->valuedouble!=i->valueint||i->valueint<0||i->valueint>max)return -1;
    *v=(uint16_t)i->valueint; return 1;
}
int parse_pid_params(cJSON *params,const char *name)
{
    PID_Controller *c=pid_by_name(name); pid_config_t cfg; cJSON *o; int n; uint16_t sepmax;
    if(!params||!c||(o=cJSON_GetObjectItem(params,name))==NULL||(o->type&0xff)!=cJSON_Object||pid_controller_get_config(c,&cfg)<0)return -1;
    sepmax=!strncmp(name,"yaw_",4)?PID_YAW_SEPARATION_MAX:PID_DEPTH_SEPARATION_MAX;
#define U(k,f,m) do{if(json_uint(o,k,&cfg.f,m)<0)return -1;}while(0)
    U("p",p,PID_GAIN_SETTING_MAX);U("i",i,PID_GAIN_SETTING_MAX);U("d",d,PID_GAIN_SETTING_MAX);U("output_limit",output_limit,PID_OUTPUT_LIMIT_MAX);U("integral_limit",integral_limit,PID_INTEGRAL_LIMIT_MAX);U("integral_separation_error",integral_separation_error,sepmax);
#undef U
#define B(k,f) do{cJSON *q=cJSON_GetObjectItem(o,k);if(q){int t=q->type&0xff;if(t==cJSON_True)n=1;else if(t==cJSON_False)n=0;else if(t==cJSON_Number&&(q->valueint==0||q->valueint==1))n=q->valueint;else return -1;cfg.f=(uint8_t)n;}}while(0)
    B("p_enabled",p_enabled);B("i_enabled",i_enabled);B("d_enabled",d_enabled);B("integral_limit_enabled",integral_limit_enabled);B("integral_separation_enabled",integral_separation_enabled);
#undef B
    return pid_controller_configure(c,&cfg);
}

static cJSON *pid_parameters_json_create(const PID_Controller *c)
{
    pid_config_t v;cJSON *o=cJSON_CreateObject();if(!o||pid_controller_get_config(c,&v)<0){cJSON_Delete(o);return NULL;}
#define N(k,f) cJSON_AddNumberToObject(o,k,v.f)
    N("p",p);N("i",i);N("d",d);N("output_limit",output_limit);N("integral_limit",integral_limit);N("integral_separation_error",integral_separation_error);
#undef N
#define B(k,f) do{if(v.f)cJSON_AddTrueToObject(o,k);else cJSON_AddFalseToObject(o,k);}while(0)
    B("p_enabled",p_enabled);B("i_enabled",i_enabled);B("d_enabled",d_enabled);B("integral_limit_enabled",integral_limit_enabled);B("integral_separation_enabled",integral_separation_enabled);
#undef B
    return o;
}

static void add_runtime(cJSON *root,const char *name,const pid_runtime_t *v,int active,int valid)
{
    cJSON *o=cJSON_CreateObject(); if(!o)return;
    if(active)cJSON_AddTrueToObject(o,"active");else cJSON_AddFalseToObject(o,"active");
    if(valid)cJSON_AddTrueToObject(o,"sensor_valid");else cJSON_AddFalseToObject(o,"sensor_valid");
    cJSON_AddNumberToObject(o,"target",v->target);cJSON_AddNumberToObject(o,"feedback",v->feedback);cJSON_AddNumberToObject(o,"error",v->error);
    cJSON_AddNumberToObject(o,"p_output",v->p_output);cJSON_AddNumberToObject(o,"i_output",v->i_output);cJSON_AddNumberToObject(o,"d_output",v->d_output);cJSON_AddNumberToObject(o,"output",v->output);
    cJSON_AddItemToObject(root,name,o);
}

cJSON *pid_runtime_status_json_create(void)
{
    auv_pid_runtime_status_t s;cJSON *o=cJSON_CreateObject();if(!o||auv_pid_runtime_status_get(&s)<0){cJSON_Delete(o);return cJSON_CreateNull();}
    cJSON_AddNumberToObject(o,"timestamp_ms",(double)s.timestamp_ms);add_runtime(o,"depth_position",&s.depth,s.depth_active,s.depth_sensor_valid);add_runtime(o,"yaw_angle",&s.yaw,s.yaw_active,s.yaw_sensor_valid);return o;
}

cJSON *rov_parameters_response_create(void)
{
    cJSON *response;
    cJSON *control_loops;
    cJSON *devices;
    cJSON *robot_arm;
    pwm_device_t arm;

    response = propeller_parameters_json_create();
    if (response == NULL)
        return NULL;

    control_loops = cJSON_CreateObject();
    devices = cJSON_CreateObject();
    robot_arm = cJSON_CreateObject();

    if (control_loops == NULL || devices == NULL || robot_arm == NULL ||
        robot_arm_get(&arm) < 0)
    {
        cJSON_Delete(control_loops);
        cJSON_Delete(devices);
        cJSON_Delete(robot_arm);
        cJSON_Delete(response);
        return NULL;
    }

    cJSON_AddItemToObject(control_loops, "depth_position",
        pid_parameters_json_create(&pid_controllers_get()->High_Position_Control));
    cJSON_AddItemToObject(control_loops, "depth_speed",
        pid_parameters_json_create(&pid_controllers_get()->High_Speed_Control));
    cJSON_AddItemToObject(control_loops, "yaw_angle",
        pid_parameters_json_create(&pid_controllers_get()->Yaw_Angle_Control));
    cJSON_AddItemToObject(control_loops, "yaw_gyro",
        pid_parameters_json_create(&pid_controllers_get()->Yaw_Gyro_Control));


    cJSON_AddNumberToObject(robot_arm, "min_pwm", arm.nMax);
    cJSON_AddNumberToObject(robot_arm, "max_pwm", arm.pMax);
    cJSON_AddNumberToObject(robot_arm, "speed", arm.speed);
    cJSON_AddItemToObject(devices, "robot_arm", robot_arm);
    cJSON_AddItemToObject(response, "control_loop_parameters", control_loops);
    cJSON_AddItemToObject(response, "devices", devices);
    return response;
}
