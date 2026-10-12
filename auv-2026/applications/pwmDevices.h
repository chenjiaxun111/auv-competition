/**
 * @file pwmDevices.h
 * @brief PWM设备的参数类型、输出范围和公共接口声明。
 *
 * 推进器接口使用微秒脉宽表示目标输出：1500us为停止值，较大或较小的
 * 数值分别表示两个转动方向。具体通道映射和PCA9685写入由pwmDevices.c完成。
 */

#ifndef __PWMDEVICES_H_
#define __PWMDEVICES_H_

#include "../user/config.h"

/*
 * 六路推进器控制量。
 * 每个成员不是绝对PWM脉宽，而是相对1500us停止值的有符号偏移量。
 */
typedef struct
{
    int16_t leftUp;
    int16_t rightUp;
    int16_t leftDown;
    int16_t rightDown;
    int16_t leftMiddle;
    int16_t rightMiddle;

} propellerPower_t;

/*
 * 发布一组完整的六路推进器目标值。
 * 六路数值未变化时不标记更新；发生变化时覆盖旧快照，等待PWM线程
 * 在下一个20ms周期取走。
 * 返回0表示处理成功（包括数值未变化），返回-1表示同步操作失败。
 */
int propeller_power_publish(const propellerPower_t *power);

/**
 * @brief 获取最近发布的六路推进器目标推力快照。
 * @param snapshot 接收完整推力快照，不得为NULL。
 * @return 成功返回0，参数无效返回-1。
 */
int propeller_power_get(propellerPower_t *snapshot);

/* 普通PWM设备参数，适用于机械臂、舵机等按脉宽控制的设备。 */
typedef struct
{
    const char *name; // 用于日志和参数文件的设备名称
    uint16_t speed;   // 每次控制周期增加或减少的脉宽步长
    uint16_t channel; // PCA9685输出通道
    uint16_t pMax;    // 最大允许脉宽
    uint16_t nMax;    // 最小允许脉宽
    uint16_t cur;     // 当前输出脉宽

} pwm_device_t;

/**
 * @brief 初始化PCA9685并创建PWM设备线程
 * @return 0成功，-1 PCA9685初始化失败
 */
int pwm_devices_init(void);
void pwm_devices_shutdown(void);

/**
 * @brief 获取机械臂PWM设备的线程安全快照，不访问硬件。
 * @param snapshot 接收当前设备状态和配置，不得为NULL。
 * @return 成功返回0，参数无效返回-1。
 */
int robot_arm_get(pwm_device_t *snapshot);

/**
 * @brief 更新机械臂的最小/最大脉宽和变化步长。
 * @param config 新配置；名称、通道和当前输出不会被复制。
 * @return 成功返回0，参数无效返回-1。
 */
int robot_arm_set_config(const pwm_device_t *config);

#endif
