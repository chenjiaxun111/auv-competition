/**
 * @file ads1118.c
 * @brief ADS1118的SPI初始化、通道选择和16位ADC数据读取。
 *
 * 驱动将四个单端输入注册为WiringPi虚拟模拟引脚。调用analogRead()时，
 * 先写入目标通道配置，等待该通道完成转换，再发送不更新配置的NOP帧读取
 * 最新结果。ADS1118返回的是有符号16位二进制补码。
 *
 * @warning 当前驱动尚未在ROV实机上完成测试。SPI通道、通信模式、MUX切换
 * 等待时间、输入量程和原始值换算均需结合实际硬件验证。在完成万用表对照、
 * 多通道切换和带载测试前，不得将采集结果用于欠压或过流保护。
 */

#define LOG_TAG "ads1118"

#include <elog.h>
#include <stdint.h>
#include <wiringPi.h>
#include <wiringPiSPI.h>

#include "ads1118.h"

#define ADS1118_SPI_CHANNEL          1
#define ADS1118_SPI_CLOCK_HZ         1000000
#define ADS1118_SPI_MODE             1
#define ADS1118_CHANNEL_COUNT        4
#define ADS1118_MUX_MASK             0x7000u
#define ADS1118_MUX_SINGLE_BASE      0x04u
#define ADS1118_CONVERSION_DELAY_MS  35
/* ADS1118可编程增益对应的双极性满量程。 */
typedef enum
{
    ADS1118_RANGE_6_144V = 0,
    ADS1118_RANGE_4_096V = 1,
    ADS1118_RANGE_2_048V = 2,
    ADS1118_RANGE_1_024V = 3,
    ADS1118_RANGE_0_512V = 4,
    ADS1118_RANGE_0_256V = 5,
} ads1118_range_t;

/*
 * 公共配置：连续转换、64 SPS、ADC模式、内部上拉开启。
 * NOP位为01，表示写入的配置有效；bit 0为数据手册要求写1的保留位。
 * MUX位由ads1118_select_channel()按通道补入。
 */
#define ADS1118_BASE_CONFIG ((0x03u << 5) | (0x01u << 3) | (0x01u << 1) | 0x01u)

static uint16_t ads1118_config;
/* 默认输入量程；需要更改上电量程时只修改此处。 */
static ads1118_range_t ads1118_range = ADS1118_RANGE_2_048V;
static const float ads1118_full_scale[] = {6.144f, 4.096f, 2.048f, 1.024f, 0.512f, 0.256f};

/**
 * @brief 按当前量程将ADC原始值换算为ADS1118输入引脚电压。
 * @param raw_value 有符号16位ADC原始值。
 * @return ADS1118输入引脚电压，单位V。
 */
float ads1118RawToVoltage(int16_t raw_value)
{
    return raw_value * (ads1118_full_scale[ads1118_range] / 32768.0f);
}

/**
 * @brief 完成一次16位SPI全双工交换。
 * @param spi_channel WiringPi SPI通道编号。
 * @param output MOSI发送的16位数据。
 * @param input MISO接收的16位数据，可为NULL。
 * @return 0成功，-1通信失败。
 */
static int ads1118_transfer16(int spi_channel, uint16_t output, uint16_t *input)
{
    uint8_t data[2] = {(uint8_t)(output >> 8), (uint8_t)output};

    if (wiringPiSPIDataRW(spi_channel, data, sizeof(data)) < 0)
    {
        log_e("ADS1118 SPI transfer failed");
        return -1;
    }

    if (input != NULL)
    {
        *input = ((uint16_t)data[0] << 8) | data[1];
    }

    return 0;
}

/** 写入目标单端通道配置。 */
static int ads1118_select_channel(int spi_channel, int channel)
{
    if (channel < 0 || channel >= ADS1118_CHANNEL_COUNT)
    {
        log_e("ADS1118 channel must be in [0, 3]");
        return -1;
    }

    ads1118_config &= ~ADS1118_MUX_MASK;
    ads1118_config |= (uint16_t)(ADS1118_MUX_SINGLE_BASE + channel) << 12;

    /* 写配置时同时收到的是此前缓存的转换结果，此处主动丢弃。 */
    return ads1118_transfer16(spi_channel, ads1118_config, NULL);
}

/**
 * @brief 读取一个单端通道的最新转换结果。
 * @param value 输出的有符号16位ADC原始值。
 */
static int ads1118_read_channel(int spi_channel, int channel, int16_t *value)
{
    uint16_t raw_value;

    if (value == NULL || ads1118_select_channel(spi_channel, channel) != 0)
    {
        return -1;
    }

    /*
     * 64 SPS时一个转换周期约15.6 ms。连续模式切换MUX时，最坏情况下需
     * 等待旧转换结束后再完成一次新通道转换，因此保守等待35 ms。
     */
    delay(ADS1118_CONVERSION_DELAY_MS);

    /* DIN全0表示不更新配置；本次16位交换只取回最新转换结果。 */
    if (ads1118_transfer16(spi_channel, 0x0000u, &raw_value) != 0)
    {
        return -1;
    }

    *value = (int16_t)raw_value;
    return 0;
}

/** WiringPi的analogRead回调，将虚拟引脚换算为ADS1118通道。 */
static int ads1118_analog_read(struct wiringPiNodeStruct *node, int pin)
{
    const int channel = pin - node->pinBase;
    int16_t value;

    if (ads1118_read_channel(node->fd, channel, &value) != 0)
    {
        return 0;
    }

    return value;
}

/**
 * @brief 初始化ADS1118使用的SPI接口并注册四个WiringPi模拟输入节点。
 * @param pin_base WiringPi虚拟引脚起始编号。
 * @return 0成功；-1表示SPI接口不可用；-2表示节点创建失败。
 *
 * @note SPI接口打开成功不能证明ADS1118一定已连接。原代码用“两路结果
 * 都为0”判断设备缺失，但0是合法ADC值，会误报，因此不再使用该判断。
 * @warning 本初始化流程及后续采样时序仍待ROV硬件测试。
 */
int ads1118Setup(const int pin_base)
{
    struct wiringPiNodeStruct *node;

    if (wiringPiSPISetupMode(ADS1118_SPI_CHANNEL, ADS1118_SPI_CLOCK_HZ, ADS1118_SPI_MODE) < 0)
    {
        return -1;
    }

    /* 根据默认量程和AIN0通道生成上电配置。 */
    ads1118_config = ADS1118_BASE_CONFIG | ((uint16_t)ads1118_range << 9) | (ADS1118_MUX_SINGLE_BASE << 12);

    /* 写入AIN0初始配置；同时返回的旧转换结果不使用。 */
    if (ads1118_transfer16(ADS1118_SPI_CHANNEL, ads1118_config, NULL) != 0)
    {
        return -1;
    }

    node = wiringPiNewNode(pin_base, ADS1118_CHANNEL_COUNT);
    if (node == NULL)
    {
        return -2;
    }

    /* node->fd保存WiringPi SPI通道编号，不是Linux文件描述符。 */
    node->fd = ADS1118_SPI_CHANNEL;
    node->analogRead = ads1118_analog_read;

    return 0;
}
