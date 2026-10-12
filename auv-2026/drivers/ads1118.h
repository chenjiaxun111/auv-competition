/**
 * @file ads1118.h
 * @brief ADS1118驱动的公开初始化接口。
 *
 * @warning 当前驱动待ROV实机测试。验证完成前，采样结果仅供调试观察，
 * 不得直接用于电池欠压、推进器过流等安全保护逻辑。
 */

#ifndef __ADS1118_H_
#define __ADS1118_H_

#include <stdint.h>

/**
 * @brief 初始化ADS1118并注册四个WiringPi虚拟模拟输入节点。
 * @param pin_base 虚拟引脚起点；pin_base到pin_base+3对应AIN0到AIN3。
 * @return 0成功，负数表示初始化失败。
 * @warning 当前SPI通信和采样流程尚未完成硬件验证。
 */
int ads1118Setup(int pin_base);

/** 按当前输入量程将有符号ADC原始值换算为ADS1118引脚电压。 */
float ads1118RawToVoltage(int16_t raw_value);

#endif
