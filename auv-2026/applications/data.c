/**
 * @file data.c
 * @brief PID、普通PWM设备和推进器参数的持久化读写。
 *
 * 本模块从parameters/control_parameters.json加载全部PID可配置项和机械臂
 * 等普通PWM设备参数，并可将运行中的当前配置写回同一文件。期望值、反馈值、
 * 误差、积分累计和历史输出属于运行时状态，不写入参数文件。
 *
 * 标准JSON不支持语法注释，文件中的_comment是供人阅读的普通字段，加载时
 * 会被忽略。推进器JSON的解析与生成仍由rpc_json模块负责，本模块只负责
 * 参数文件持久化。
 */

#define LOG_TAG "data"

#include <elog.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "data.h"
#include "pwmDevices.h"
#include "../connections/cjson.h"
#include "../connections/rovRpcData.h"
#include "../control/PID.h"
#include "../user/config.h"

#define CONTROL_CONFIG_FILE_PATH "./parameters/control_parameters.json"
#define CONTROL_CONFIG_TEMP_FILE_PATH "./parameters/control_parameters.json.tmp"
#define PROPELLER_CONFIG_FILE_PATH "./parameters/propeller_parameters.json"
#define PROPELLER_CONFIG_TEMP_FILE_PATH "./parameters/propeller_parameters.json.tmp"

/**
 * @brief 创建指定PID控制器的JSON对象并加入controllers。
 * @param controllers control_parameters.json中的controllers对象。
 * @param name 控制器在JSON中的名称。
 * @param controller PID控制器参数来源。
 */
static void add_pid_json(cJSON *controllers, const char *name, const PID_Controller *controller)
{
    cJSON *pid;
    pid_config_t cfg;

    if (controllers == NULL || name == NULL || controller == NULL ||
        pid_controller_get_config(controller, &cfg) < 0)
        return;

    pid = cJSON_CreateObject();
    if (pid == NULL)
        return;

    cJSON_AddNumberToObject(pid, "p", cfg.p); cJSON_AddNumberToObject(pid, "i", cfg.i); cJSON_AddNumberToObject(pid, "d", cfg.d);
    cJSON_AddNumberToObject(pid, "p_enabled", cfg.p_enabled); cJSON_AddNumberToObject(pid, "i_enabled", cfg.i_enabled); cJSON_AddNumberToObject(pid, "d_enabled", cfg.d_enabled);
    cJSON_AddNumberToObject(pid, "integral_limit_enabled", cfg.integral_limit_enabled);
    cJSON_AddNumberToObject(pid, "integral_separation_enabled", cfg.integral_separation_enabled);
    cJSON_AddNumberToObject(pid, "integral_separation_error", cfg.integral_separation_error);
    cJSON_AddNumberToObject(pid, "integral_limit", cfg.integral_limit);
    cJSON_AddNumberToObject(pid, "output_limit", cfg.output_limit);
    cJSON_AddItemToObject(controllers, name, pid);
}

/**
 * @brief 根据RAM中的当前参数创建完整控制参数JSON。
 *
 * 生成PID控制器和普通PWM设备参数，用于保存到control_parameters.json。
 *
 * @return 成功时返回根JSON对象，创建失败时返回NULL。
 */
static cJSON *create_control_config_json(void)
{
    cJSON *root = cJSON_CreateObject();
    cJSON *controllers = cJSON_CreateObject();
    cJSON *devices = cJSON_CreateObject();
    cJSON *robot_arm = cJSON_CreateObject();
    pwm_device_t arm;

    if (root == NULL || controllers == NULL || devices == NULL || robot_arm == NULL)
    {
        cJSON_Delete(root);
        cJSON_Delete(controllers);
        cJSON_Delete(devices);
        cJSON_Delete(robot_arm);
        return NULL;
    }

    add_pid_json(controllers, "yaw_angle", &pid_controllers_get()->Yaw_Angle_Control);
    add_pid_json(controllers, "yaw_gyro", &pid_controllers_get()->Yaw_Gyro_Control);
    add_pid_json(controllers, "depth_position", &pid_controllers_get()->High_Position_Control);
    add_pid_json(controllers, "depth_speed", &pid_controllers_get()->High_Speed_Control);

    if (robot_arm_get(&arm) == 0)
    {
        cJSON_AddNumberToObject(robot_arm, "min_pwm", arm.nMax);
        cJSON_AddNumberToObject(robot_arm, "max_pwm", arm.pMax);
        cJSON_AddNumberToObject(robot_arm, "speed", arm.speed);
    }

    cJSON_AddStringToObject(robot_arm, "_comment", "机械臂普通PWM设备参数");
    cJSON_AddItemToObject(devices, "robot_arm", robot_arm);
    cJSON_AddStringToObject(root, "_comment", "说明字段仅供阅读，程序只读取controllers和devices中的参数值");
    cJSON_AddItemToObject(root, "controllers", controllers);
    cJSON_AddItemToObject(root, "devices", devices);
    return root;
}

/**
 * @brief 从JSON对象读取一个必需的数值字段。
 * @param object 包含目标字段的JSON对象。
 * @param name 数值字段名称。
 * @param value 用于接收读取结果的地址。
 * @return 成功返回0，字段不存在或不是数值时返回-1。
 */
static int read_number(cJSON *object, const char *name, double *value)
{
    cJSON *item = cJSON_GetObjectItem(object, name);
    if (item == NULL || item->type != cJSON_Number)
        return -1;
    *value = item->valuedouble;
    return 0;
}

/**
 * @brief 校验并把一组JSON PID参数加载到RAM中的控制器。
 * @param controllers control_parameters.json中的controllers对象。
 * @param name 需要读取的控制器名称。
 * @param controller 用于接收全部可配置参数的PID控制器。
 * @return 所有必需字段合法时返回0，否则返回-1且记录错误日志。
 */
static int apply_pid_json(cJSON *controllers, const char *name,PID_Controller *controller)
{
    cJSON *pid = cJSON_GetObjectItem(controllers, name);
    pid_config_t cfg = {0}; double v[11]; int i;
    const char *keys[]={"p","i","d","output_limit","integral_limit","integral_separation_error","p_enabled","i_enabled","d_enabled","integral_limit_enabled","integral_separation_enabled"};

    if (pid == NULL || (pid->type & 0xFF) != cJSON_Object)
    {
        log_w("Invalid or missing PID parameters: %s", name);
        return -1;
    }

    for (i = 0; i < 11; ++i) {
        if (read_number(pid, keys[i], &v[i]) != 0 || v[i] < 0 ||
            v[i] != (uint16_t)v[i]) {
            log_w("Invalid or missing PID parameter %s.%s", name, keys[i]);
            return -1;
        }
    }
    if (v[0] > PID_GAIN_SETTING_MAX || v[1] > PID_GAIN_SETTING_MAX ||
        v[2] > PID_GAIN_SETTING_MAX || v[3] > PID_OUTPUT_LIMIT_MAX ||
        v[4] > PID_INTEGRAL_LIMIT_MAX ||
        v[5] > (name[0] == 'y' ? PID_YAW_SEPARATION_MAX :
                                 PID_DEPTH_SEPARATION_MAX)) return -1;
    for (i = 6; i < 11; ++i) if (v[i] > 1) return -1;
    cfg.p = (uint16_t)v[0]; cfg.i = (uint16_t)v[1]; cfg.d = (uint16_t)v[2];
    cfg.output_limit = (uint16_t)v[3]; cfg.integral_limit = (uint16_t)v[4];
    cfg.integral_separation_error = (uint16_t)v[5];
    cfg.p_enabled = (uint8_t)v[6]; cfg.i_enabled = (uint8_t)v[7];
    cfg.d_enabled = (uint8_t)v[8]; cfg.integral_limit_enabled = (uint8_t)v[9];
    cfg.integral_separation_enabled = (uint8_t)v[10];
    return pid_controller_configure(controller,&cfg);
}

/**
 * @brief 校验并应用机械臂普通PWM设备参数。
 * @param devices control_parameters.json中的devices对象。
 *
 * 参数无效时保留RAM中的现有机械臂配置并记录警告。
 */
static void apply_robot_arm_json(cJSON *devices)
{
    cJSON *robot_arm = cJSON_GetObjectItem(devices, "robot_arm");
    pwm_device_t arm;
    double min_pwm, max_pwm, speed;

    if (robot_arm == NULL || robot_arm->type != cJSON_Object ||
        read_number(robot_arm, "min_pwm", &min_pwm) != 0 ||
        read_number(robot_arm, "max_pwm", &max_pwm) != 0 ||
        read_number(robot_arm, "speed", &speed) != 0 ||
        min_pwm < 0 || max_pwm > UINT16_MAX || speed < 0 ||
        speed > UINT16_MAX || min_pwm > max_pwm)
    {
        log_w("Invalid or missing robot-arm parameters");
        return;
    }

    if (robot_arm_get(&arm) == 0)
    {
        arm.nMax = (uint16_t)min_pwm;
        arm.pMax = (uint16_t)max_pwm;
        arm.speed = (uint16_t)speed;
        if (robot_arm_set_config(&arm) < 0)
            log_e("Failed to apply robot-arm parameters");
    }
}

/**
 * @brief 将指定文本文件完整读取到动态分配的字符串中。
 * @param path 要读取的文件路径。
 * @return 成功时返回以'\0'结尾的字符串，失败时返回NULL；调用者负责free。
 */
static char *read_text_file(const char *path)
{
    FILE *fp = fopen(path, "rb");
    long length;
    char *text;

    if (fp == NULL || fseek(fp, 0, SEEK_END) != 0)
    {
        if (fp != NULL) fclose(fp);
        return NULL;
    }

    length = ftell(fp);
    if (length < 0 || fseek(fp, 0, SEEK_SET) != 0)
    {
        fclose(fp);
        return NULL;
    }

    text = malloc((size_t)length + 1);
    if (text == NULL)
    {
        fclose(fp);
        return NULL;
    }

    if (fread(text, 1, (size_t)length, fp) != (size_t)length)
    {
        free(text);
        fclose(fp);
        return NULL;
    }

    text[length] = '\0';
    fclose(fp);
    return text;
}

/**
 * @brief 从parameters目录加载PID和普通PWM设备参数。
 *
 * 读取并解析control_parameters.json，要求十组PID控制器的全部配置字段合法；
 * 成功后将参数写入RAM。说明字段会被忽略。
 *
 * @return 全部必需参数加载成功返回0，读取、解析或校验失败返回-1。
 */
int pid_and_pwm_device_parameters_load(void)
{
    char *text = read_text_file(CONTROL_CONFIG_FILE_PATH);
    cJSON *root;
    cJSON *controllers;
    cJSON *devices;

    if (text == NULL)
    {
        log_e("Failed to read PID and PWM-device parameters from %s",
              CONTROL_CONFIG_FILE_PATH);
        return -1;
    }

    root = cJSON_Parse(text);
    free(text);
    if (root == NULL)
    {
        log_e("Invalid JSON in PID and PWM-device parameter file: %s",
              CONTROL_CONFIG_FILE_PATH);
        return -1;
    }

    controllers = cJSON_GetObjectItem(root, "controllers");
    devices = cJSON_GetObjectItem(root, "devices");
    if (controllers == NULL || controllers->type != cJSON_Object)
    {
        log_e("Missing controllers object in parameter file: %s",
              CONTROL_CONFIG_FILE_PATH);
        cJSON_Delete(root);
        return -1;
    }

    if (apply_pid_json(controllers, "yaw_angle", &pid_controllers_get()->Yaw_Angle_Control) < 0 ||
        apply_pid_json(controllers, "yaw_gyro", &pid_controllers_get()->Yaw_Gyro_Control) < 0 ||
        apply_pid_json(controllers, "depth_position", &pid_controllers_get()->High_Position_Control) < 0 ||
        apply_pid_json(controllers, "depth_speed", &pid_controllers_get()->High_Speed_Control) < 0)
    {
        cJSON_Delete(root);
        return -1;
    }
    if (devices != NULL && devices->type == cJSON_Object)
        apply_robot_arm_json(devices);

    cJSON_Delete(root);
    log_i("Loaded PID and PWM-device parameters from %s",
          CONTROL_CONFIG_FILE_PATH);
    return 0;
}

/**
 * @brief 将RAM中的PID和普通PWM设备参数保存到parameters目录。
 *
 * 写入control_parameters.json时会同时生成中文控制器名称和参数说明字段。
 */
int pid_and_pwm_device_parameters_save(void)
{
    cJSON *root = create_control_config_json();
    char *text;
    FILE *fp;

    if (root == NULL)
    {
        log_e("Failed to create control parameter JSON");
        return -1;
    }

    text = cJSON_Print(root);
    cJSON_Delete(root);
    if (text == NULL)
        return -1;

    fp = fopen(CONTROL_CONFIG_TEMP_FILE_PATH, "w");
    if (fp == NULL)
    {
        free(text);
        log_e("Failed to open control parameter file");
        return -1;
    }

    if (fputs(text, fp) == EOF || fputc('\n', fp) == EOF)
    {
        free(text);
        fclose(fp);
        remove(CONTROL_CONFIG_TEMP_FILE_PATH);
        log_e("Failed to write control parameters");
        return -1;
    }
    free(text);
    if (fclose(fp) != 0)
    {
        remove(CONTROL_CONFIG_TEMP_FILE_PATH);
        log_e("Failed to close control parameter file");
        return -1;
    }
    if (rename(CONTROL_CONFIG_TEMP_FILE_PATH, CONTROL_CONFIG_FILE_PATH) != 0)
    {
        remove(CONTROL_CONFIG_TEMP_FILE_PATH);
        log_e("Failed to replace control parameter file");
        return -1;
    }
    return 0;
}

/**
 * @brief 从parameters目录加载六路推进器参数。
 *
 * 读取并解析propeller_parameters.json，校验根对象和propeller_parameters
 * 对象后，将参数交给RPC JSON解析层写入运行状态。
 *
 * @return 读取并应用成功返回0，文件或JSON无效时返回-1。
 */
int propeller_parameters_load(void)
{
    char *text = read_text_file(PROPELLER_CONFIG_FILE_PATH);
    cJSON *root;
    cJSON *propellers;

    if (text == NULL)
    {
        log_e("Unable to read propeller parameter file: %s",
              PROPELLER_CONFIG_FILE_PATH);
        return -1;
    }

    root = cJSON_Parse(text);
    free(text);
    propellers = root == NULL
        ? NULL
        : cJSON_GetObjectItem(root, "propeller_parameters");

    if (root == NULL || (root->type & 0xFF) != cJSON_Object ||
        propellers == NULL || (propellers->type & 0xFF) != cJSON_Object)
    {
        cJSON_Delete(root);
        log_e("Invalid propeller parameter file: %s",
              PROPELLER_CONFIG_FILE_PATH);
        return -1;
    }

    if (update_propeller_parameters(propellers) < 0)
    {
        log_e("推进器参数文件包含无效数值");
        cJSON_Delete(root);
        return -1;
    }
    cJSON_Delete(root);
    log_i("Loaded propeller parameters from %s", PROPELLER_CONFIG_FILE_PATH);
    return 0;
}

/**
 * @brief 将RAM中的六路推进器参数保存到parameters目录。
 *
 * 先写临时文件，完整写入并关闭成功后再替换正式文件，避免保存中断损坏原配置。
 */
int propeller_parameters_save(void)
{
    cJSON *root = propeller_parameters_json_create();
    char *text;
    FILE *fp;
    size_t length;
    int write_failed;

    if (root == NULL)
    {
        log_e("Unable to create propeller parameter JSON");
        return -1;
    }

    text = cJSON_Print(root);
    cJSON_Delete(root);
    if (text == NULL)
    {
        log_e("Unable to format propeller parameters");
        return -1;
    }

    fp = fopen(PROPELLER_CONFIG_TEMP_FILE_PATH, "wb");
    if (fp == NULL)
    {
        free(text);
        log_e("Unable to open temporary propeller parameter file");
        return -1;
    }

    length = strlen(text);
    write_failed = fwrite(text, 1, length, fp) != length;
    if (!write_failed && fputc('\n', fp) == EOF)
        write_failed = 1;
    if (fclose(fp) != 0)
        write_failed = 1;
    free(text);

    if (write_failed)
    {
        remove(PROPELLER_CONFIG_TEMP_FILE_PATH);
        log_e("Unable to write propeller parameters");
        return -1;
    }

    if (rename(PROPELLER_CONFIG_TEMP_FILE_PATH, PROPELLER_CONFIG_FILE_PATH) != 0)
    {
        remove(PROPELLER_CONFIG_TEMP_FILE_PATH);
        log_e("Unable to replace propeller parameter file");
        return -1;
    }

    log_i("Saved propeller parameters to %s", PROPELLER_CONFIG_FILE_PATH);
    return 0;
}
