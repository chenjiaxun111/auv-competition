/**
 * @file propellerTask.c
 * @brief 推进器控制任务的创建、周期调度和目标值发布。
 *
 * 控制线程周期读取上位机摇杆命令，调用控制算法计算六路推进器目标值，
 * 再将最新结果发布给PWM设备线程。本模块只产生控制目标，不直接操作
 * PCA9685硬件；退出时由主线程等待本线程结束并回收资源。
 */

#define LOG_TAG "propeller_task"

#include <elog.h>
#include <pthread.h>
#include <stdatomic.h>
#include <string.h>
#include <wiringPi.h>

#include "../user/config.h"
#include "../control/control.h"
#include "../control/auvControl.h"
#include "../connections/rovCommandState.h"

#include "propellerTask.h"

/*
 * 推进器控制线程ID。
 * 使用模块级静态变量保存，便于后续增加线程停止、状态查询等功能；
 * static 限制该变量只能在本文件内访问，避免其他模块直接操作线程。
 */
static pthread_t propeller_tid;
static atomic_bool propeller_thread_running = ATOMIC_VAR_INIT(0);
static int propeller_thread_started;

/**
 * @brief 推进器控制线程
 *
 * 周期性读取上位机运动指令，依次完成深度控制、航向控制、
 * 水平推进器混控和六路推力合成。计算结果保存在
 * 线程局部推力快照中，再通过最新值邮箱通知PWM设备线程输出到PCA9685。
 *
 * @param arg 线程参数，当前未使用
 * @return 收到停止请求后返回NULL
 */
static void *propeller_thread(void *arg)
{
    propellerPower_t propeller_power;
    rockerInfo_t rocker;

    (void)arg;

    while (atomic_load(&propeller_thread_running))
    {
        get_rocker_params(&rocker);                   //读取并转换上位机运动指令
#if VEHICLE_MODEL == VEHICLE_MODEL_AUV
        auv_control(&rocker,
            rov_command_state_heartbeat_alive(AUV_HEARTBEAT_TIMEOUT_MS));
#endif
        
        rov_vertical_control(&rocker);                //根据Z轴手柄生成两台垂直推进器推力
                
        rov_horizontal_control(&rocker);              //计算四台水平推进器推力

        rov_propeller_output(&propeller_power);       //合成六路目标推力

        /* 将完整六路推力发布到最新值邮箱，并通知PWM线程进行硬件输出。 */
        if (propeller_power_publish(&propeller_power) != 0)
            log_e("Failed to publish propeller power");
        
        delay(AUV_CONTROL_PERIOD_MS);                 //固定50Hz控制周期
    }

    return NULL;
}

/**
 * @brief 初始化推进器控制线程
 *
 * 创建一个可连接的后台控制线程。退出时由
 * propeller_task_shutdown()通知线程停止，并调用pthread_join()等待其结束。
 *
 * @return 0 初始化成功，-1 线程创建失败
 */
int propeller_task_init(void)
{
    int result;

    if (!PROPELLER_ENABLED)
    {
        log_i("Propeller controller disabled");
        return 0;
    }

    atomic_store(&propeller_thread_running, 1);

    /* 创建线程，propeller_tid接收线程ID，当前不向线程传递参数。 */
    result = pthread_create(&propeller_tid, NULL, propeller_thread, NULL);
    if (result != 0)
    {
        atomic_store(&propeller_thread_running, 0);
        log_e("Failed to create propeller controller thread: %s", strerror(result));
        return -1;
    }

    propeller_thread_started = 1;

    log_i("Propeller controller initialized");
    return 0;
}

/** 停止产生新的推进器目标值。 */
void propeller_task_shutdown(void)
{
    int result;

    if (!propeller_thread_started)
        return;

    atomic_store(&propeller_thread_running, 0);

    result = pthread_join(propeller_tid, NULL);
    if (result != 0)
        log_e("Failed to join propeller controller thread: %s", strerror(result));

    propeller_thread_started = 0;
}
