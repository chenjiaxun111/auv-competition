/**
 * @file pca9685.c
 * @brief PCA9685的I2C初始化、PWM频率配置和WiringPi扩展节点驱动。
 *
 * 本模块通过user/config.h配置的I2C总线访问PCA9685，并将其16个PWM通道注册
 * 为一组WiringPi虚拟引脚。上层调用pwmWrite(虚拟引脚, 计数值)时，
 * WiringPi会转入本文件的myPwmWrite()，最终写入对应通道的LED寄存器。
 */

#define LOG_TAG "pca9685"

#include <elog.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>

#include "pca9685.h"
#include "../user/config.h"

/* 本驱动使用的PCA9685寄存器地址。 */
#define PCA9685_MODE1 0x00
#define PCA9685_PRESCALE 0xFE
#define LED0_ON_L 0x06
#define LEDALL_ON_L 0xFA
#define LEDALL_OFF_L 0xFC

/* MODE1和PWM通道寄存器的位掩码。 */
#define MODE1_RESTART_BIT 0x80
#define MODE1_AUTO_INCREMENT_BIT 0x20
#define MODE1_SLEEP_BIT 0x10
#define PWM_FULL_BIT 0x10
#define PWM_VALUE_MASK 0x0FFF
#define PWM_FULL_WORD 0x1000 // 两个连续寄存器合并后，bit12对应高字节的FULL位
#define PWM_RESOLUTION 4096  // 12位PWM计数器每个周期的计数份数

/* WiringPi虚拟通道16代表芯片的ALL_LED寄存器组。 */
#define PIN_ALL 16

#define PCA9685_FREQ_MIN 40
#define PCA9685_FREQ_MAX 1000
#define PCA9685_FREQ_DEFAULT 50

/**
 * @brief 根据PCA9685通道号取得该通道LED_ON_L寄存器地址
 * @param pin 0～15表示单通道，PIN_ALL表示全部通道
 * @return 该通道寄存器组的起始地址
 *
 * PCA9685的每个PWM通道连续占用4个8位寄存器：
 *     LEDx_ON_L、LEDx_ON_H   ：保存高电平开始位置的12位计数值；
 *     LEDx_OFF_L、LEDx_OFF_H ：保存高电平结束位置的12位计数值。
 * 因此相邻通道的起始地址相差4，第x路的起始地址为：
 *     LED0_ON_L + 4 * x
 * 例如通道2的起始地址是0x06 + 4 * 2 = 0x0E，对应LED2_ON_L。
 *
 * PIN_ALL是本驱动定义的特殊通道号（16），它不是第17路物理输出。
 * 传入PIN_ALL时返回芯片自带的ALL_LED寄存器组，一次操作全部16路。
 */
static int baseReg(int pin)
{
    /* 单通道按每路4个寄存器计算地址；PIN_ALL选择ALL_LED寄存器组。 */
    return (pin >= PIN_ALL ? LEDALL_ON_L : LED0_ON_L + 4 * pin);
}

/**
  * @brief 使用ALL_LED寄存器将全部PWM通道强制关闭
  * @param fd PCA9685的I2C文件描述符
  */
static void pca9685PWMReset(int fd)
{
    wiringPiI2CWriteReg16(fd, LEDALL_ON_L, 0x0000);  // 清除ALL_LED的FULL ON位，避免所有通道被强制为高电平
    wiringPiI2CWriteReg16(fd, LEDALL_OFF_L, PWM_FULL_WORD); // 设置FULL OFF，强制全部通道关闭
}

/**
  * @brief 设置一个PCA9685通道的高电平起点和终点计数
  * @param fd PCA9685的I2C文件描述符
  * @param pin PCA9685通道号0～15，或PIN_ALL
  * @param on 高电平开始计数，范围0～4095
  * @param off 高电平结束计数，范围0～4095
  *
  * ON_H和OFF_H的bit4分别是FULL ON和FULL OFF。本函数只写入
  * on/off的低12位，因此会同时把两个FULL位清0，让通道恢复为
  * 根据ON/OFF计数值生成波形的普通PWM模式。
  */
static void pca9685PWMWrite(int fd, int pin, int on, int off)
{
    int reg = baseReg(pin);

    /* ON和OFF均为独立12位计数值，并不要求两者之和等于4095。 */
    wiringPiI2CWriteReg16(fd, reg, on & PWM_VALUE_MASK);
    wiringPiI2CWriteReg16(fd, reg + 2, off & PWM_VALUE_MASK);
}

/**
  * @brief 设置或清除指定通道的FULL OFF位
  * @param fd PCA9685的I2C文件描述符
  * @param pin PCA9685通道号0～15，或PIN_ALL
  * @param tf 非0强制持续低电平，0恢复普通PWM控制
  *
  * PCA9685对FULL ON/FULL OFF的解释：
  *     FULL ON=0，FULL OFF=0：按ON/OFF计数值输出普通PWM；
  *     FULL ON=1，FULL OFF=0：整个周期保持ON状态；
  *     FULL ON=0，FULL OFF=1：整个周期保持OFF状态；
  *     FULL ON=1，FULL OFF=1：FULL OFF优先，最终保持OFF状态。
  * 这里的ON/OFF是芯片官方的逻辑状态名称；若配置了输出反相，
  * 它们不一定分别对应物理高、低电平。
  */
static void pca9685FullOff(int fd, int pin, int tf)
{
    int reg = baseReg(pin) + 3; // LEDX_OFF_H 寄存器
    int state = wiringPiI2CReadReg8(fd, reg);

    /* OFF_H的bit4是FULL OFF控制位。 */
    state = tf ? (state | PWM_FULL_BIT) : (state & ~PWM_FULL_BIT);

    wiringPiI2CWriteReg8(fd, reg, state);
}

/**
  * @brief 设置或清除指定通道的FULL ON位
  * @param fd PCA9685的I2C文件描述符
  * @param pin PCA9685通道号0～15，或PIN_ALL
  * @param tf 非0强制持续高电平，0恢复普通PWM控制
  */
static void pca9685FullOn(int fd, int pin, int tf)
{
    int reg = baseReg(pin) + 1; // LEDX_ON_H 寄存器
    int state = wiringPiI2CReadReg8(fd, reg);

    /* ON_H的bit4是FULL ON控制位。 */
    state = tf ? (state | PWM_FULL_BIT) : (state & ~PWM_FULL_BIT);

    wiringPiI2CWriteReg8(fd, reg, state);

    //  full-off 优先级高于 full-on (datasheet P23)
    if (tf)
        pca9685FullOff(fd, pin, 0);
}

/**
  * @brief WiringPi调用的PCA9685 PWM写入回调
  * @param node PCA9685对应的WiringPi扩展节点
  * @param pin WiringPi虚拟引脚编号
  * @param value 高电平结束计数：小于等于0为全关，1～4095为PWM，
  *              大于等于4096为全开
  */
static void myPwmWrite(struct wiringPiNodeStruct *node, int pin, int value)
{
    int fd = node->fd;
    int ipin = pin - node->pinBase; // 虚拟引脚转换为PCA9685通道号

    if (value >= PWM_RESOLUTION) pca9685FullOn(fd, ipin, 1);
    else if (value > 0) pca9685PWMWrite(fd, ipin, 0, value);
    else pca9685FullOff(fd, ipin, 1);
}

/**
  * @brief WiringPi调用的数字量写入回调
  * @param node PCA9685对应的WiringPi扩展节点
  * @param pin WiringPi虚拟引脚编号
  * @param value 0输出持续低电平，非0输出持续高电平
  *
  * 该函数只在上层对PCA9685虚拟引脚调用digitalWrite()时，由
  * WiringPi通过node->digitalWrite回调。它与pwmWrite()的调用链无关：
  *     digitalWrite(300, HIGH) -> myOnOffWrite() -> 通道0 FULL ON
  *     digitalWrite(300, LOW)  -> myOnOffWrite() -> 通道0 FULL OFF
  * 当前ROV的推进器和机械臂只调用pwmWrite()，没有对虚拟引脚
  * 300～316调用digitalWrite()；保留此回调是为了使驱动同时支持
  * WiringPi的数字量输出接口。
  */
static void myOnOffWrite(struct wiringPiNodeStruct *node, int pin, int value)
{
    int fd = node->fd;
    int ipin = pin - node->pinBase;

    if (value) pca9685FullOn(fd, ipin, 1);
    else pca9685FullOff(fd, ipin, 1);
}

/**
  * @brief 配置PCA9685所有通道共用的PWM频率
  * @param fd PCA9685的I2C文件描述符
  * @param freq 目标频率，函数内部限制到40～1000Hz
  *
  * 修改PRE_SCALE前必须先进入SLEEP模式，写入后唤醒并执行RESTART。
  * 预分频值按数据手册的标准公式计算，不再使用额外经验修正系数。
  */
static void pca9685PWMSetFreq(int fd, float freq)
{
    /*
     * 在驱动内部完成限幅，不依赖某个WiringNP版本是否提供constrain符号，
     * 避免不同NanoPi系统在最终链接时出现undefined reference。
     */
    if (freq < PCA9685_FREQ_MIN)
        freq = PCA9685_FREQ_MIN;
    else if (freq > PCA9685_FREQ_MAX)
        freq = PCA9685_FREQ_MAX;

    /* 手册公式：prescale = round(osc_clock / (4096 * frequency)) - 1。 */
    int prescale = (int)(PCA9685_OSC_CLK / ((float)PWM_RESOLUTION * freq) + 0.5f) - 1;

    /* 基于当前MODE1值生成休眠、唤醒和重启三个状态。 */
    int settings = wiringPiI2CReadReg8(fd, PCA9685_MODE1) & ~MODE1_RESTART_BIT;
    int sleep = settings | MODE1_SLEEP_BIT;
    int wake = settings & ~MODE1_SLEEP_BIT;
    int restart = wake | MODE1_RESTART_BIT;

    /* 进入休眠后修改PRE_SCALE，再唤醒振荡器。 */
    wiringPiI2CWriteReg8(fd, PCA9685_MODE1, sleep);
    wiringPiI2CWriteReg8(fd, PCA9685_PRESCALE, prescale);
    wiringPiI2CWriteReg8(fd, PCA9685_MODE1, wake);

    /* 等待内部振荡器稳定后重新启动PWM计数器。 */
    delay(1);
    wiringPiI2CWriteReg8(fd, PCA9685_MODE1, restart);
}

/**
 * @brief 初始化PCA9685并注册WiringPi虚拟引脚节点
 * @param pinBase 虚拟引脚基址，应避开板载GPIO和其他扩展节点的编号范围
 * @param freq 目标PWM频率，允许范围40～1000Hz；非法值回退到50Hz
 * @return 成功时返回I2C文件描述符；-1表示I2C接口打开失败，-2表示芯片
 *         寄存器读取失败，-4表示WiringPi扩展节点创建失败
 *
 * 初始化顺序：使能OE输出、打开I2C设备、探测芯片、开启寄存器自动递增、
 * 设置PWM频率、创建17个虚拟引脚（16个通道加1个ALL通道）、注册回调，
 * 最后关闭全部PWM输出。
 */
int pca9685Setup(int pinBase, float freq)
{   
    /*
     * fd是Linux文件描述符。成功打开/dev/i2c-0后，Linux返回一个非负整数，
     * 后续所有I2C寄存器读写都通过该fd指定同一条总线和同一个从设备。
     */
    int fd;
    int settings, autoInc;
    /* WiringPi扩展节点用于把PCA9685通道映射为一组虚拟GPIO编号。 */
    struct wiringPiNodeStruct *node = NULL;

    /*
     * PCA9685_OE_PIN是NanoPi自身的普通GPIO，不是PCA9685的PWM通道。
     * 它连接PCA9685的OE（Output Enable）引脚，只负责控制16路输出总开关。
     */
    pinMode(PCA9685_OE_PIN, OUTPUT);
    /* OE低电平有效：LOW允许16路PWM输出，HIGH会同时关闭全部输出。 */
    digitalWrite(PCA9685_OE_PIN, LOW);

    /*
     * PCA9685_I2C_DEVICE是实机检测到PCA9685的I2C总线，0x40是PCA9685在
     * 该总线上的从设备地址。这里的“文件”是内核访问硬件的接口，
     * 不是保存文字内容的普通文件。
     */
    fd = wiringPiI2CSetupInterface(PCA9685_I2C_DEVICE, PCA9685_I2C_ADDRESS);
    /* 返回负数表示I2C总线不存在、未启用、权限不足或设备打开失败。 */
    if (fd < 0)
        return -1;

    /*
     * 尝试读取MODE1配置寄存器，以确认地址0x40的芯片可以正常应答。
     * 读取成功时settings保存寄存器的8位内容；返回负数通常表示芯片未接、
     * 地址错误、接线异常或I2C通信失败。
     */
    if ((settings = wiringPiI2CReadReg8(fd, PCA9685_MODE1)) < 0)
        return -2;

    /*
     * MODE1的bit5是自动递增开关。置1以后，连续读写多个字节时寄存器地址
     * 可以自加；同时清除bit7的RESTART位，避免在此误触发恢复。
     */
    autoInc = (settings | MODE1_AUTO_INCREMENT_BIT) & ~MODE1_RESTART_BIT;
    /* 将修改后的MODE1值通过I2C写回PCA9685。 */
    wiringPiI2CWriteReg8(fd, PCA9685_MODE1, autoInc);

    if (freq < PCA9685_FREQ_MIN || freq > PCA9685_FREQ_MAX)
    {
        log_e("PCA9685 freq range in [40, 1000]");
        freq = PCA9685_FREQ_DEFAULT;
    }
        
    /* 所有16个通道共用同一个PWM频率；当前上层传入50Hz。 */
    pca9685PWMSetFreq(fd, freq);

    /*
     * 创建WiringPi虚拟引脚节点。例如pinBase为300时：300～315分别映射
     * PCA9685通道0～15，316映射一次控制全部通道的ALL_LED寄存器。
     */
    node = wiringPiNewNode(pinBase, PIN_ALL + 1);
    if (!node)
        return -4;

    /*
     * 保存I2C文件描述符并注册回调。以后上层执行pwmWrite(300, value)时，
     * WiringPi会找到该节点并自动调用myPwmWrite(node, 300, value)。
     */
    node->fd = fd;
    node->pwmWrite = myPwmWrite;
    node->digitalWrite = myOnOffWrite; // 注册digitalWrite()回调；当前ROV未使用，作为通用驱动能力保留

    /* 初始化完成时先关闭全部输出，避免设备产生意外动作。 */
    pca9685PWMReset(fd);

    return fd;
}
