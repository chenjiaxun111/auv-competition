/**
 * @file pwmDevices.c
 * @brief PCA9685 PWM设备驱动与输出线程。
 *
 * 本文件负责初始化PCA9685，并把推进器和机械臂的目标值
 * 转换为硬件PWM输出。推进器控制线程通过单槽“最新值邮箱”发布六路目标值，
 * 统一PWM线程每20ms检查推进器更新，并按100ms周期处理机械臂。所有
 * PCA9685写入都由同一线程完成，避免多个线程同时访问I2C设备。
 */

#define LOG_TAG "pwm_devices"

#include <elog.h>
#include <pthread.h>
#include <stdatomic.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <wiringPi.h>

#include "pwmDevices.h"

#include "data.h"
#include "../connections/rovCommandState.h"
#include "../drivers/pca9685.h"

/* PCA9685与推进器输出参数，仅供本文件使用。 */
#define HERTZ 50             // PCA9685输出频率，50Hz对应20ms周期
#define MAX_PWM 4096         // PCA9685将一个PWM周期划分为4096个计数

#define PROPELLER_POWER_P_MAX 2000 // 推进器允许的最大脉宽，单位us
#define PROPELLER_POWER_STOP 1500  // 推进器停止脉宽，单位us
#define PROPELLER_POWER_N_MAX 1000 // 推进器允许的最小脉宽，单位us

#define PROPELLER_COUNT 6

/*
 * 数组顺序与propellerPwm_update()中的六个逻辑推进器严格对应。
 * 实际PCA9685通道只需要在user/config.h中修改。
 */
static const int propeller_channels[PROPELLER_COUNT] = {
    PROPELLER_RIGHT_UP_CHANNEL,
    PROPELLER_RIGHT_MIDDLE_CHANNEL,
    PROPELLER_RIGHT_DOWN_CHANNEL,
    PROPELLER_LEFT_UP_CHANNEL,
    PROPELLER_LEFT_MIDDLE_CHANNEL,
    PROPELLER_LEFT_DOWN_CHANNEL,
};

/* PWM输出线程的运行周期配置。 */
#define ARM_UPDATE_PERIODS   5  // 机械臂每5个基础周期更新一次，即100ms

/* 普通PWM设备动作命令，仅供本文件的设备处理流程使用。 */
#define PWM_ACTION_HOLD     0x00 // 保持当前目标脉宽
#define PWM_ACTION_INCREASE 0x01 // 增大目标脉宽
#define PWM_ACTION_DECREASE 0x02 // 减小目标脉宽

/*
 * 推进器最新值邮箱：控制线程发布完整快照，PWM线程周期检查并取走。
 * 新数据会覆盖尚未处理的旧数据，避免PWM线程执行过期控制命令。
 */
/* 保护邮箱内的推力快照和数据有效标志，防止复制过程中被另一线程改写。 */
static pthread_mutex_t propeller_power_mutex = PTHREAD_MUTEX_INITIALIZER;
/* 邮箱中保存的最新六路推力；始终整组复制，不逐成员跨线程读写。 */
static propellerPower_t latest_propeller_power;
/* 1表示邮箱中有尚未输出的新数据，0表示本周期无需更新推进器。 */
static int propeller_power_available = 0;
static pthread_mutex_t robot_arm_mutex = PTHREAD_MUTEX_INITIALIZER;
static pwm_device_t robot_arm = {
    .name = "robot-arm",
    .pMax = 2300,
    .nMax = 700,
    .speed = 80,
    .channel = ROBOT_ARM_CHANNEL,
};

/**
 * @brief 线程安全地读取最近一次发布的六路推进器目标推力。
 * @param snapshot 用于接收完整推进器推力快照的地址。
 * @return 成功返回0，snapshot为NULL时返回-1。
 *
 * 本函数只复制RAM中的最新目标值，不读取PCA9685，也不改变邮箱的数据有效标志。
 */
int propeller_power_get(propellerPower_t *snapshot)
{
    if (snapshot == NULL) return -1;
    pthread_mutex_lock(&propeller_power_mutex);
    *snapshot = latest_propeller_power;
    pthread_mutex_unlock(&propeller_power_mutex);
    return 0;
}

/**
 * @brief 线程安全地读取机械臂PWM设备当前状态和配置。
 * @param snapshot 用于接收机械臂设备快照的地址。
 * @return 成功返回0，snapshot为NULL时返回-1。
 *
 * 快照包含当前脉宽、最小/最大脉宽、变化步长和输出通道。本函数不访问硬件。
 */
int robot_arm_get(pwm_device_t *snapshot)
{
    if (snapshot == NULL) return -1;
    pthread_mutex_lock(&robot_arm_mutex);
    *snapshot = robot_arm;
    pthread_mutex_unlock(&robot_arm_mutex);
    return 0;
}

/**
 * @brief 线程安全地更新机械臂可配置的PWM参数。
 * @param config 包含最小脉宽、最大脉宽和变化步长的新配置。
 * @return 成功返回0，config为NULL时返回-1。
 *
 * 本函数不修改设备名称、PCA9685通道和当前输出，也不会立即写入硬件；新配置
 * 会在机械臂初始化或后续PWM线程更新输出时生效。
 */
int robot_arm_set_config(const pwm_device_t *config)
{
    if (config == NULL) return -1;
    pthread_mutex_lock(&robot_arm_mutex);
    robot_arm.nMax = config->nMax;
    robot_arm.pMax = config->pMax;
    robot_arm.speed = config->speed;
    pthread_mutex_unlock(&robot_arm_mutex);
    return 0;
}

/*
 * PWM设备工作线程ID。
 * 保存在模块级静态变量中，便于以后在本模块内增加状态查询、停止或
 * 资源管理功能；static使其他文件不能直接修改这些线程ID。
 */
static pthread_t pwm_devices_tid;
static atomic_bool pwm_devices_thread_running = ATOMIC_VAR_INIT(0);
static int pwm_devices_thread_started;

/** 检查PCA9685通道范围、推进器重复占用及机械臂通道冲突。 */
static int pwm_channel_config_validate(void)
{
    int i;
    int j;

    if (PROPELLER_ENABLED)
    {
        for (i = 0; i < PROPELLER_COUNT; ++i)
        {
            if (propeller_channels[i] < 0 || propeller_channels[i] > 15)
            {
                log_e("Invalid propeller PWM channel: %d",
                      propeller_channels[i]);
                return -1;
            }

            for (j = i + 1; j < PROPELLER_COUNT; ++j)
            {
                if (propeller_channels[i] == propeller_channels[j])
                {
                    log_e("Duplicate propeller PWM channel: %d",
                          propeller_channels[i]);
                    return -1;
                }
            }

            if (ROBOT_ARM_ENABLED &&
                propeller_channels[i] == ROBOT_ARM_CHANNEL)
            {
                log_e("Robot arm and propeller share PWM channel: %d",
                      propeller_channels[i]);
                return -1;
            }
        }
    }

    if (ROBOT_ARM_ENABLED &&
        (ROBOT_ARM_CHANNEL < 0 || ROBOT_ARM_CHANNEL > 15))
    {
        log_e("Invalid robot-arm PWM channel: %d", ROBOT_ARM_CHANNEL);
        return -1;
    }

    return 0;
}

/**
 * @brief 比较两组六路推进器目标值是否完全相同
 * @param left 第一组推进器目标值
 * @param right 第二组推进器目标值
 * @return 1 完全相同，0 至少一路不同
 */
static int propeller_power_equal(const propellerPower_t *left, const propellerPower_t *right)
{
    return left->leftUp == right->leftUp &&
           left->rightUp == right->rightUp &&
           left->leftDown == right->leftDown &&
           left->rightDown == right->rightDown &&
           left->leftMiddle == right->leftMiddle &&
           left->rightMiddle == right->rightMiddle;
}

/**
 * @brief 将最新六路推进器控制量放入单槽邮箱
 * @param power 六路推进器相对1500us停止值的偏移量
 * @return 0 处理成功（已发布或数值未变化），-1 参数或线程同步操作失败
 *
 * 本函数由推进器控制线程调用。六路数据在互斥锁内整组复制，尚未被PWM
 * 线程读取的旧快照允许被覆盖，因为执行最新控制量比保留历史控制量重要。
 */
int propeller_power_publish(const propellerPower_t *power)
{
    /* 拒绝无效地址，避免在复制结构体时发生空指针访问。 */
    if (power == NULL)
        return -1;

    /* 进入临界区，保证“复制推力”和“设置有效标志”构成一个完整操作。 */
    int result = pthread_mutex_lock(&propeller_power_mutex);
    if (result != 0)
        return -1;

    /*
     * 六路推力均未变化时直接返回。PCA9685会持续输出上一次写入值，
     * 因此不需要重复标记更新或写入相同寄存器。
     */
    if (propeller_power_equal(power, &latest_propeller_power))
    {
        pthread_mutex_unlock(&propeller_power_mutex);
        return 0;
    }

    /*
     * 覆盖旧快照而不是排队保存：推进器只需要最新目标值，
     * 尚未输出的旧控制量不应在之后继续执行。
     */
    latest_propeller_power = *power;
    propeller_power_available = 1;

    /*
     * 统一PWM线程会在下一个20ms周期读取该快照，因此这里只记录
     * “存在新数据”的状态，不需要额外发送条件变量通知。
     */
    pthread_mutex_unlock(&propeller_power_mutex);
    return 0;
}

/**
 * @brief 非阻塞地尝试取得最新六路推进器控制量
 * @param power 用于保存六路控制量的线程局部快照
 * @return 1 取得新数据，0 当前没有新数据，-1 参数或互斥锁操作失败
 *
 * 本函数只由统一PWM输出线程调用。没有新数据时立即返回，不会重复写入
 * PCA9685；调用线程随后继续处理机械臂等普通PWM设备。
 */
static int propeller_power_try_receive(propellerPower_t *power)
{
    int result;

    /* 接收缓存必须有效。 */
    if (power == NULL)
        return -1;

    /* 检查状态和复制快照必须在同一临界区完成。 */
    result = pthread_mutex_lock(&propeller_power_mutex);
    if (result != 0)
        return -1;

    if (!propeller_power_available)
    {
        pthread_mutex_unlock(&propeller_power_mutex);
        return 0;
    }

    /* 在锁内复制完整六路快照，避免取得新旧控制周期混合的数据。 */
    *power = latest_propeller_power;
    /* 标记本次更新已取走；后续周期等待控制线程再次发布。 */
    propeller_power_available = 0;

    /* 快照已保存到PWM线程的局部变量，可以释放邮箱锁。 */
    pthread_mutex_unlock(&propeller_power_mutex);
    return 1;
}

/**
 * @brief 将目标脉宽从微秒换算为PCA9685的计数值
 * @param pulse_width_us 高电平持续时间，单位us，例如1500us
 * @return PCA9685一个PWM周期内的高电平计数值
 *
 * PCA9685把一个完整PWM周期划分为MAX_PWM（4096）份。频率为50Hz时，
 * 一个周期为20ms；1500us即1.5ms，因此理论计数值为：
 *     1.5ms / 20ms * 4096 = 307.2
 * 返回值取整数后写入PCA9685。
 */
static int calcTicks(int16_t pulse_width_us)
{
    const float pwm_period_ms = 1000.0f / HERTZ;
    const float pulse_width_ms = pulse_width_us / 1000.0f;
    const float theoretical_ticks = pulse_width_ms / pwm_period_ms * MAX_PWM;

    return (int)theoretical_ticks;
}

/**
 * @brief 将六路推进器绝对脉宽限制在电调允许范围内
 * @param val 含6个元素的PWM脉宽数组，单位us；函数会直接修改数组
 */
static void propellerPwm_output_limit(int16_t *val)
{
    int i;
    for (i = 0; i < PROPELLER_COUNT; i++)
    {
        val[i] = constrain(val[i], PROPELLER_POWER_N_MAX, PROPELLER_POWER_P_MAX);
    }
}

/**
 * @brief 将普通PWM设备的当前脉宽限制在配置范围内
 * @param device PWM设备参数
 */
static void pwm_device_output_limit(pwm_device_t *device)
{
    device->cur = constrain(device->cur, device->nMax, device->pMax);
}

/**
 * @brief 将普通PWM设备的初始输出设置为配置的最小脉宽。
 */
static void pwm_device_init(pwm_device_t *device)
{
    device->cur = device->nMax;
    pwmWrite(PCA9685_PIN_BASE + device->channel, calcTicks(device->cur));
}

/**
 * @brief 初始化六路推进器的安全PWM输出
 *
 * M080手册没有规定额外解锁流程，因此当前只输出1500us停止信号。
 * 原机器人使用过的“1550us保持500ms，再回到1500us保持1s”流程保留
 * 在函数内作为注释参考，不参与编译和运行。
 */
static void propeller_init(void)
{
    int i;

    /*
     * 原电调解锁流程（当前禁用）：
     *
     * for (i = 0; i < PROPELLER_COUNT; i++)
     *     pwmWrite(PCA9685_PIN_BASE + propeller_channels[i], calcTicks(1550));
     *
     * delay(500);
     *
     * for (i = 0; i < PROPELLER_COUNT; i++)
     *     pwmWrite(PCA9685_PIN_BASE + propeller_channels[i],
     *              calcTicks(PROPELLER_POWER_STOP));
     *
     * sleep(1);
     */

    /* 当前流程：六路电调只接收1500us安全停止信号。 */
    for (i = 0; i < PROPELLER_COUNT; i++)
        pwmWrite(PCA9685_PIN_BASE + propeller_channels[i],
                 calcTicks(PROPELLER_POWER_STOP));
}

/**
 * @brief 将六路推进器控制量转换为绝对脉宽并写入PCA9685
 * @param propeller 六路推进器相对1500us停止值的偏移量
 *
 * 通道映射由user/config.h中的六个PROPELLER_*_CHANNEL宏决定。
 * 换算后的绝对脉宽会先限制到1000～2000us，再转换为PCA9685计数值。
 */
static void propellerPwm_update(const propellerPower_t *propeller)
{
    int16_t power[PROPELLER_COUNT];

    power[0] = PROPELLER_POWER_STOP + propeller->rightUp;
    power[1] = PROPELLER_POWER_STOP + propeller->rightMiddle;
    power[2] = PROPELLER_POWER_STOP + propeller->rightDown;

    power[3] = PROPELLER_POWER_STOP + propeller->leftUp;
    power[4] = PROPELLER_POWER_STOP + propeller->leftMiddle;
    power[5] = PROPELLER_POWER_STOP + propeller->leftDown;

    propellerPwm_output_limit(power);
    for (int i = 0; i < PROPELLER_COUNT; i++)
    {
        pwmWrite(PCA9685_PIN_BASE + propeller_channels[i],
                 calcTicks(power[i]));
    }

    log_i("PWM RU(CH%d)=%dus/%dt RM(CH%d)=%dus/%dt RD(CH%d)=%dus/%dt "
          "LU(CH%d)=%dus/%dt LM(CH%d)=%dus/%dt LD(CH%d)=%dus/%dt",
          propeller_channels[0], power[0], calcTicks(power[0]),
          propeller_channels[1], power[1], calcTicks(power[1]),
          propeller_channels[2], power[2], calcTicks(power[2]),
          propeller_channels[3], power[3], calcTicks(power[3]),
          propeller_channels[4], power[4], calcTicks(power[4]),
          propeller_channels[5], power[5], calcTicks(power[5]));
}

/**
 * @brief 根据动作指令更新一个普通PWM设备并输出脉宽
 * @param device PWM设备参数
 * @param action PWM_ACTION_HOLD、PWM_ACTION_INCREASE或PWM_ACTION_DECREASE
 *
 * 每处理一次指令，当前脉宽变化speed微秒。指令处理后清零，随后执行
 * 输出限幅并写入配置的PCA9685通道。
 */
static void pwm_device_update(pwm_device_t *device, uint8_t *action)
{
    switch (*action)
    {
    case PWM_ACTION_INCREASE:
        device->cur += device->speed; // 正向
        break;
    case PWM_ACTION_DECREASE:
        device->cur -= device->speed; // 反向
        break;
    default:
        break;
    }
    *action = PWM_ACTION_HOLD; // 本次动作已处理，避免下一周期重复使用旧指令

    pwm_device_output_limit(device);
    pwmWrite(PCA9685_PIN_BASE + device->channel, calcTicks(device->cur));
}

/**
 * @brief 统一处理推进器和普通PWM设备的PCA9685输出线程
 *
 * 本线程是PCA9685唯一写入者。它以20ms为基础周期：推进器有新快照时
 * 立即写入六路通道；每5个基础周期（100ms）处理一次机械臂。推进器
 * 控制计算仍在独立线程中运行，不受此次输出线程合并影响。
 *
 * @param arg 线程参数，当前未使用
 * @return 收到停止请求并完成推进器安全输出后返回NULL
 */
static void *pwm_devices_thread(void *arg)
{
    propellerPower_t power_snapshot;
    rov_command_state_t rpc_snapshot;
    const propellerPower_t stop_power = {0};
    uint8_t arm_action;
    int arm_period_count = 0;
    int receive_result;

    (void)arg;

    while (atomic_load(&pwm_devices_thread_running))
    {
        if (PROPELLER_ENABLED)
        {
            receive_result = propeller_power_try_receive(&power_snapshot);
            if (receive_result > 0)
                propellerPwm_update(&power_snapshot);
            else if (receive_result < 0)
                log_e("Failed to receive propeller power");
        }

        arm_period_count++;
        if (arm_period_count >= ARM_UPDATE_PERIODS)
        {
            arm_period_count = 0;

            if (ROBOT_ARM_ENABLED)
            {
                rov_command_state_get_snapshot(&rpc_snapshot);
                if (rpc_snapshot.gripper_input > 0) arm_action = PWM_ACTION_INCREASE;
                else if (rpc_snapshot.gripper_input < 0) arm_action = PWM_ACTION_DECREASE;
                else arm_action = PWM_ACTION_HOLD;
                pthread_mutex_lock(&robot_arm_mutex);
                pwm_device_update(&robot_arm, &arm_action);
                pthread_mutex_unlock(&robot_arm_mutex);
            }

            /* 其他普通PWM设备的周期处理可继续添加在这里。 */
        }

        delay(20);
    }

    /* 本线程是PCA9685唯一写入者，退出前直接输出六路安全停止值。 */
    if (PROPELLER_ENABLED)
        propellerPwm_update(&stop_power);

    return NULL;
}

/**
 * @brief 初始化PCA9685并创建统一PWM输出线程
 * @return 0 初始化成功，-1 PCA9685初始化失败
 *
 * PCA9685建立成功后，分别根据PROPELLER_ENABLED和ROBOT_ARM_ENABLED
 * 初始化推进器与机械臂的安全输出，再创建唯一的可连接
 * PWM输出线程。退出时由pwm_devices_shutdown()等待线程完成安全停机。
  */
int pwm_devices_init(void)
{
    static int fd;
    int result;

    if (!PROPELLER_ENABLED && !ROBOT_ARM_ENABLED)
    {
        log_i("PWM devices disabled");
        return 0;
    }

    if (pwm_channel_config_validate() < 0)
        return -1;

    fd = pca9685Setup(PCA9685_PIN_BASE, HERTZ);
    /* I2C接口或PCA9685不可用时，不创建任何PWM输出线程。 */
    if (fd < 0)
    {
        ERROR_LOG(fd, "PCA9685");
        return -1;
    }

    log_i("PCA9685 init");

    if (PROPELLER_ENABLED)
    {
        /* 创建输出线程前先完成电调启动流程。 */
        propeller_init();
    }

    if (ROBOT_ARM_ENABLED)
    {
        /* 与推进器一样，创建线程前先输出机械臂安全初始值。 */
        pthread_mutex_lock(&robot_arm_mutex);
        pwm_device_init(&robot_arm);
        pthread_mutex_unlock(&robot_arm_mutex);
    }

    atomic_store(&pwm_devices_thread_running, 1);
    result = pthread_create(&pwm_devices_tid, NULL, pwm_devices_thread, NULL);
    if (result != 0)
    {
        atomic_store(&pwm_devices_thread_running, 0);
        log_e("Failed to create PWM devices thread: %s", strerror(result));
        return -1;
    }

    pwm_devices_thread_started = 1;

    return 0;
}

/**
 * @brief 请求PWM线程退出，并等待六路推进器回到停止位。
 */
void pwm_devices_shutdown(void)
{
    int result;

    if (!pwm_devices_thread_started)
        return;

    atomic_store(&pwm_devices_thread_running, 0);

    result = pthread_join(pwm_devices_tid, NULL);
    if (result != 0)
        log_e("Failed to join PWM devices thread: %s", strerror(result));

    pwm_devices_thread_started = 0;
}
