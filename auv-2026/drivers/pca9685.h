/**
 * @file pca9685.h
 * @brief PCA9685驱动的公开初始化接口。
 */

#ifndef PCA9685_H
#define PCA9685_H

/**
 * @brief 初始化PCA9685并向WiringPi注册虚拟引脚
 * @param pinBase WiringPi虚拟引脚起始编号
 * @param freq 所有PCA9685通道共用的PWM频率，单位Hz
 * @return 成功返回I2C文件描述符，失败返回负数错误码
 */
int pca9685Setup(int pinBase, float freq);

#endif
