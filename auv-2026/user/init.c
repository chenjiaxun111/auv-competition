/**
 * @file init.c
 * @brief ROV系统模块初始化与安全停机。
 *
 * 本文件按启动顺序初始化日志、板级接口、
 * 传感器、PCA9685 PWM设备、状态检测、推进器控制线程和上位机通信服务。
 * 初始化失败时应由相应模块返回错误，避免在硬件未就绪时继续启动控制。
 */
#define LOG_TAG "init"

#include <errno.h>
#include <elog.h>
#include <string.h>
#include <wiringPi.h>

#include "../applications/propellerTask.h"
#include "../applications/data.h"
#include "../applications/statusLight.h"
#include "../applications/pwmDevices.h"
#include "../applications/sensor.h"
#include "../applications/server.h"
#include "config.h"
#include "init.h"


/**
 * @brief  下位机系统初始化
 */
int rov_init(void)
{
    easylogger_init(); // easylogger日志系统 初始化

    if (wiringPiSetup() < 0) //判断NanoPi 是否运行成功
    {
        log_e("Unable to start wiringPi: %s", strerror(errno));
        return -1;
    }
    log_i("Welcome to AUV Master V%s\n", AUV_MASTER_VERSION);

    /* 蓝灯：等待上位机连接。 */
    status_light_init();

    /* 直接从parameters/control_parameters.json加载PID和普通PWM设备参数。 */
    if (pid_and_pwm_device_parameters_load() < 0)
    {
        status_light_set(STATUS_LIGHT_FAULT);
        return -1;
    }

    /* 从parameters/propeller_parameters.json加载六路推进器参数。 */
    if (propeller_parameters_load() < 0)
    {
        status_light_set(STATUS_LIGHT_FAULT);
        return -1;
    }

    sensor_thread_init();  // 传感器线程 初始化

    if (pwm_devices_init() < 0) // 无安全PWM输出时不得继续启动推进器控制
    {
        status_light_set(STATUS_LIGHT_FAULT);
        return -1;
    }

    if (propeller_task_init() < 0) // 根据配置启动推进器控制任务
    {
        pwm_devices_shutdown();
        status_light_set(STATUS_LIGHT_FAULT);
        return -1;
    }
    
    if (server_thread_init() < 0) // RPC线程初始化
    {
        propeller_task_shutdown();
        pwm_devices_shutdown();
        status_light_set(STATUS_LIGHT_FAULT);
        return -1;
    }

    return 0;
}

/**
 * @brief 停止控制计算，并在退出前将推进器置为零推力。
 */
void rov_shutdown(void)
{
    propeller_task_shutdown();
    pwm_devices_shutdown();
    log_i("System shutdown complete");
}
