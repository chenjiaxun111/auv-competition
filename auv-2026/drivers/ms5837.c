/**
 * @desc: MS5837 深度传感器驱动
 *
 *       Notes: 水深传感器设备驱动
 *   Attention: SCL (黑色)   
 *				SDA (黄色)   
 */

#define LOG_TAG "ms5837"

#include <elog.h>
#include <math.h>
#include <stdio.h>
#include <unistd.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <wiringSerial.h>

#include "ms5837.h"
#include "../user/config.h"

#define MS5837_UART_FRAME_BUFFER_SIZE 32
#define MS5837_I2C_ADDRESS            0x76
#define MS5837_COMMAND_RESET          0x1E
#define MS5837_COMMAND_ADC_READ       0x00
#define MS5837_COMMAND_PROM_READ      0xA0
#define MS5837_COMMAND_D1_OSR_8192    0x4A
#define MS5837_COMMAND_D2_OSR_8192    0x5A
#define MS5837_CONVERSION_DELAY_MS    20
#define MS5837_PROM_WORD_COUNT        8

/*
 * 当前ROV使用MS5837串口解算板，下面的I2C直连驱动尚未经过实机测试。
 * 启用MS5837_IIC前，必须确认传感器为MS5837-30BA，并核对供电、地址、
 * PROM CRC、压力单位以及整机防水结构，不能直接用于比赛任务。
 */
static uint16_t ms5837_prom[MS5837_PROM_WORD_COUNT];
static float ms5837_pressure_mbar;
static float ms5837_temperature_c;

/**
  * @brief  MS5837 PROM校准参数，crc4校验函数 (datasheet P12)
  * @param   *n_prom prom中的出厂校准参数
  * @retval 返回crc 4bit校验
  */
static uint8_t ms5837_crc4(const uint16_t *prom)
{
    uint16_t prom_copy[MS5837_PROM_WORD_COUNT];
    uint16_t remainder = 0;
    int byte_index;
    int bit_index;

    for (byte_index = 0; byte_index < MS5837_PROM_WORD_COUNT; byte_index++)
    {
        prom_copy[byte_index] = prom[byte_index];
    }

    /* 手册规定计算CRC时清除PROM[0]的CRC位，并令虚拟PROM[7]为0。 */
    prom_copy[0] &= 0x0FFF;
    prom_copy[7] = 0;

    for (byte_index = 0; byte_index < 16; byte_index++)
    {
        if ((byte_index & 1) != 0)
        {
            remainder ^= prom_copy[byte_index >> 1] & 0x00FF;
        }
        else
        {
            remainder ^= prom_copy[byte_index >> 1] >> 8;
        }

        for (bit_index = 0; bit_index < 8; bit_index++)
        {
            if ((remainder & 0x8000) != 0)
            {
                remainder = (remainder << 1) ^ 0x3000;
            }
            else
            {
                remainder <<= 1;
            }
        }
    }

    return (uint8_t)((remainder >> 12) & 0x0F);
}

/**
  * @brief  ms5837获取出厂标定参数
  * @param  fd 文件描述符
  * @retval 返回出厂标定参数 crc校验 是否成功标志：1成功，-1失败
  *  若成功表示为ms5837传感器
  *  若失败表示为其他类型传感器或无
  */
static int ms5837_get_calib_param(int fd)
{
    int register_value;
    int i;

    for (i = 0; i <= 6; i++)
    {
        register_value = wiringPiI2CReadReg16(fd, MS5837_COMMAND_PROM_READ + (i * 2));
        if (register_value < 0)
        {
            return -1;
        }

        /* wiringPi按主机字节序返回16位值，MS5837的PROM数据高字节在前。 */
        ms5837_prom[i] = (uint16_t)(((uint16_t)register_value << 8) |
                                    ((uint16_t)register_value >> 8));
    }

    ms5837_prom[7] = 0;

    if ((uint8_t)(ms5837_prom[0] >> 12) != ms5837_crc4(ms5837_prom))
    {
        return -1;
    }

    return 0;
}

/**
 * @brief  ms5837 获取转换数据
 * @param  fd 文件描述符
 * @param  command  带精度温度命令  带精度温度压力(见头文件)
 * @retval 数据结果
 */
static int ms5837_get_conversion(int fd, uint8_t command, uint32_t *value)
{
    uint8_t bytes[3];

    if (value == NULL || wiringPiI2CWrite(fd, command) < 0)
    {
        return -1;
    }

    /* OSR 8192的最大转换时间为18.08 ms，等待20 ms后再读取ADC。 */
    delay(MS5837_CONVERSION_DELAY_MS);

    if (wiringPiI2CWrite(fd, MS5837_COMMAND_ADC_READ) < 0)
    {
        return -1;
    }

    if (read(fd, bytes, sizeof(bytes)) != (ssize_t)sizeof(bytes))
    {
        return -1;
    }

    *value = ((uint32_t)bytes[0] << 16) |
             ((uint32_t)bytes[1] << 8) |
             (uint32_t)bytes[2];

    return 0;
}

/**
 * @brief  获取并计算温度值，此时的温度值还没经过补偿，并不准确。
 * @param  fd 文件描述符
 */
static int ms5837_read_measurement(int fd)
{
    uint32_t raw_pressure;
    uint32_t raw_temperature;
    int32_t temperature;
    int32_t temperature_difference;
    int64_t offset;
    int64_t sensitivity;
    int64_t temperature_correction;
    int64_t offset_correction;
    int64_t sensitivity_correction;
    int64_t difference_squared;

    if (ms5837_get_conversion(fd, MS5837_COMMAND_D2_OSR_8192, &raw_temperature) < 0)
    {
        return -1;
    }

    if (ms5837_get_conversion(fd, MS5837_COMMAND_D1_OSR_8192, &raw_pressure) < 0)
    {
        return -1;
    }

    temperature_difference = (int32_t)raw_temperature - ((int32_t)ms5837_prom[5] << 8);
    temperature = 2000 + (int32_t)(((int64_t)temperature_difference * ms5837_prom[6]) >> 23);
    offset = ((int64_t)ms5837_prom[2] << 16) +
             (((int64_t)ms5837_prom[4] * temperature_difference) >> 7);
    sensitivity = ((int64_t)ms5837_prom[1] << 15) +
                  (((int64_t)ms5837_prom[3] * temperature_difference) >> 8);

    difference_squared = (int64_t)temperature_difference * temperature_difference;

    if (temperature < 2000)
    {
        int64_t low_temperature_squared = (int64_t)(temperature - 2000) *
                                          (temperature - 2000);

        temperature_correction = (3 * difference_squared) >> 33;
        offset_correction = (3 * low_temperature_squared) >> 1;
        sensitivity_correction = (5 * low_temperature_squared) >> 3;

        if (temperature < -1500)
        {
            int64_t very_low_temperature_squared = (int64_t)(temperature + 1500) *
                                                   (temperature + 1500);

            offset_correction += 7 * very_low_temperature_squared;
            sensitivity_correction += 4 * very_low_temperature_squared;
        }
    }
    else
    {
        int64_t high_temperature_squared = (int64_t)(temperature - 2000) *
                                           (temperature - 2000);

        temperature_correction = difference_squared >> 36;
        offset_correction = high_temperature_squared >> 4;
        sensitivity_correction = 0;
    }

    offset -= offset_correction;
    sensitivity -= sensitivity_correction;

    ms5837_temperature_c = (temperature - temperature_correction) / 100.0f;
    ms5837_pressure_mbar = (float)((((int64_t)raw_pressure * sensitivity) >> 21) - offset) /
                             81920.0f;

    return 0;
}

/**
 * @brief 通过I2C读取一组完整的温度和压力数据。
 * @param fd ms5837_IIC_setup()返回的I2C文件描述符。
 * @param temperature 成功时输出温度，单位℃。
 * @param pressure 成功时输出绝对压力，单位Pa。
 * @return 1表示成功，-1表示参数或I2C读取失败。
 */
int ms5837_IIC_data(int fd, float *temperature, float *pressure)
{
    if (fd < 0 || temperature == NULL || pressure == NULL)
    {
        return -1;
    }

    if (ms5837_read_measurement(fd) < 0 ||
        !isfinite(ms5837_temperature_c) || !isfinite(ms5837_pressure_mbar))
    {
        return -1;
    }

    *temperature = ms5837_temperature_c;
    *pressure = ms5837_pressure_mbar * 100.0f;

    return 1;
}

/**
 * @brief 接收一个解算板串口字节，并解析完整温度、深度数据帧。
 *
 * ROVMAKER V1.0解算板手册规定输出格式为
 * “Depth:XX.XXm Temp:XX.XXC\r\n”。本函数逐字节组装一行数据，
 * 严格验证字段顺序、单位和结束符，并支持正负浮点数。
 *
 * @param data 本次接收的一个串口字节。
 * @param temperature 成功解析后保存温度，单位℃。
 * @param depth 成功解析后保存深度，单位m。
 * @return 1表示成功解析一帧，0表示仍在等待后续字节，-1表示当前帧无效。
 */
int ms5837_usart_data(uint8_t data, float *temperature, float *depth)
{
    static char frame[MS5837_UART_FRAME_BUFFER_SIZE];
    static size_t received_length;
    float parsed_temperature;
    float parsed_depth;
    int parsed_length = 0;
    int matched_fields;

    if (temperature == NULL || depth == NULL)
    {
        return -1;
    }

    /* 协议以Depth开头；忽略行首之前的噪声。 */
    if (received_length == 0)
    {
        if (data == 'D')
        {
            frame[0] = (char)data;
            received_length = 1;
        }
        return 0;
    }

    if (received_length >= sizeof(frame) - 1)
    {
        received_length = 0;
        return -1;
    }

    frame[received_length] = (char)data;
    received_length++;

    if (data != '\n')
    {
        return 0;
    }

    if (received_length < 2 || frame[received_length - 2] != '\r')
    {
        received_length = 0;
        return -1;
    }

    /* 去掉帧尾CRLF后，只解析正文并确认没有多余字符。 */
    frame[received_length - 2] = '\0';
    matched_fields = sscanf(frame, "Depth:%fm Temp:%fC%n",
                            &parsed_depth, &parsed_temperature, &parsed_length);

    if (matched_fields != 2 || parsed_length != (int)received_length - 2 ||
        !isfinite(parsed_temperature) || !isfinite(parsed_depth))
    {
        received_length = 0;
        return -1;
    }

    *temperature = parsed_temperature;
    *depth = parsed_depth;
    received_length = 0;

    return 1;
}

/**
  * @brief  ms5837 复位
  * @param  fd 文件描述符
  */
static int ms5837_reset(int fd)
{
    return wiringPiI2CWrite(fd, MS5837_COMMAND_RESET);
}

/**
 * @brief 初始化MS5837-30BA的I2C直连驱动。
 * @param pinBase WiringPi扩展节点起始编号，必须避开已有节点。
 * @return 成功返回I2C文件描述符；失败返回负数错误码。
 *
 * @warning 当前ROV实际使用串口解算板，本I2C路径仅按官方手册完成代码
 * 核对和静态整理，尚未连接实物验证。启用前必须完成实机测试。
 */
int ms5837_IIC_setup(const int pinBase)
{
    int fd;

    /* 保留pinBase参数以兼容现有调用接口，I2C驱动不再创建虚拟GPIO节点。 */
    (void)pinBase;

    fd = wiringPiI2CSetupInterface(MS5837_I2C_DEVICE, MS5837_I2C_ADDRESS);
    if (fd < 0)
    {
        return -1;
    }

    if (ms5837_reset(fd) < 0)
    {
        close(fd);
        return -2;
    }

    /* 手册要求复位后至少等待2.8 ms，5 ms留出少量裕量。 */
    delay(5);

    if (ms5837_get_calib_param(fd) != 0)
    {
        close(fd);
        return -3;
    }

    return fd;
}

/**
 * @brief 打开当前ROV使用的MS5837串口解算板。
 * @return 成功返回串口文件描述符，失败返回-1。
 */
int ms5837_usart_setup(void)
{
    int fd;

    fd = serialOpen(MS5837_UART_DEVICE, MS5837_UART_BAUD);
    if (fd < 0)
    {
        return -1;
    }

    return fd;
}
