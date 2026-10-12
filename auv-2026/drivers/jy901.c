/**
 * @file jy901.c
 * @brief JY901串口初始化与标准11字节数据包解析。
 *
 * JY901默认主动输出数据。本驱动只负责打开串口、校验数据包，并解析当前
 * ROV实际使用的加速度、角速度、姿态角、磁场及气压/高度数据。配置、校准
 * 和恢复出厂设置应由人工维护工具完成，不在ROV每次启动时自动执行。
 */

#include <stdint.h>
#include <stddef.h>
#include <wiringSerial.h>

#include "../user/config.h"
#include "jy901.h"

#define JY901_FRAME_HEADER      0x55
#define JY901_FRAME_LENGTH      11
#define JY901_PAYLOAD_OFFSET    2
#define JY901_CHECKSUM_INDEX    (JY901_FRAME_LENGTH - 1)

#define JY901_PACKET_ACC        0x51
#define JY901_PACKET_GYRO       0x52
#define JY901_PACKET_ANGLE      0x53
#define JY901_PACKET_MAG        0x54
#define JY901_PACKET_PRESSURE   0x56

#define JY901_ACC_SCALE         (16.0f / 32768.0f)
#define JY901_GYRO_SCALE        (2000.0f / 32768.0f)
#define JY901_ANGLE_SCALE       (180.0f / 32768.0f)
#define JY901_TEMPERATURE_SCALE (1.0f / 100.0f)
#define JY901_HEIGHT_SCALE      (1.0f / 100.0f)

/** 从JY901小端字节序数据中读取一个有符号16位整数。 */
static int16_t jy901_read_i16(const uint8_t *data)
{
    uint16_t value;

    value = (uint16_t)data[0] | ((uint16_t)data[1] << 8);

    return (int16_t)value;
}

/** 从JY901小端字节序数据中读取一个有符号32位整数。 */
static int32_t jy901_read_i32(const uint8_t *data)
{
    uint32_t value;

    value = (uint32_t)data[0];
    value |= (uint32_t)data[1] << 8;
    value |= (uint32_t)data[2] << 16;
    value |= (uint32_t)data[3] << 24;

    return (int32_t)value;
}

/**
 * @brief 按照数据包类型转换有效载荷并更新JY901状态。
 * @param frame 已通过校验的11字节JY901数据包。
 * @param jy901 JY901状态存储位置。
 */
static void jy901_parse_frame(const uint8_t *frame, jy901_t *jy901)
{
    const uint8_t *payload = &frame[JY901_PAYLOAD_OFFSET];

    switch (frame[1])
    {
        case JY901_PACKET_ACC:
            jy901->acc.x = jy901_read_i16(&payload[0]) * JY901_ACC_SCALE;
            jy901->acc.y = jy901_read_i16(&payload[2]) * JY901_ACC_SCALE;
            jy901->acc.z = jy901_read_i16(&payload[4]) * JY901_ACC_SCALE;
            jy901->temperature = jy901_read_i16(&payload[6]) * JY901_TEMPERATURE_SCALE;
            break;

        case JY901_PACKET_GYRO:
            jy901->gyro.x = jy901_read_i16(&payload[0]) * JY901_GYRO_SCALE;
            jy901->gyro.y = jy901_read_i16(&payload[2]) * JY901_GYRO_SCALE;
            jy901->gyro.z = jy901_read_i16(&payload[4]) * JY901_GYRO_SCALE;
            jy901->temperature = jy901_read_i16(&payload[6]) * JY901_TEMPERATURE_SCALE;
            break;

        case JY901_PACKET_ANGLE:
            jy901->roll = jy901_read_i16(&payload[0]) * JY901_ANGLE_SCALE;
            jy901->pitch = jy901_read_i16(&payload[2]) * JY901_ANGLE_SCALE;
            jy901->yaw = jy901_read_i16(&payload[4]) * JY901_ANGLE_SCALE;
            jy901->temperature = jy901_read_i16(&payload[6]) * JY901_TEMPERATURE_SCALE;
            break;

        case JY901_PACKET_MAG:
            jy901->mag.x = jy901_read_i16(&payload[0]);
            jy901->mag.y = jy901_read_i16(&payload[2]);
            jy901->mag.z = jy901_read_i16(&payload[4]);
            jy901->temperature = jy901_read_i16(&payload[6]) * JY901_TEMPERATURE_SCALE;
            break;

        case JY901_PACKET_PRESSURE:
            jy901->pressure = (float)jy901_read_i32(&payload[0]);
            jy901->altitude = jy901_read_i32(&payload[4]) * JY901_HEIGHT_SCALE;
            break;

        default:
            /* 当前ROV不使用时间、端口状态、GPS和四元数数据包。 */
            break;
    }
}

/**
 * @brief 接收一个JY901串口字节，组装、校验并解析完整数据包。
 *
 * 本函数不是串口中断回调。jy901_thread()从Linux串口接收缓冲区逐字节
 * 读取数据后调用本函数。数据包格式为：0x55、类型、8字节数据、校验和；
 * 校验和是前10个字节相加后保留低8位。
 *
 * @param data 本次读取的一个串口字节。
 * @param jy901 解析结果的存储位置。
 */
int copeJY901_data(uint8_t data, jy901_t *jy901)
{
    static uint8_t frame[JY901_FRAME_LENGTH];
    static uint8_t received_length;
    uint8_t checksum = 0;
    uint8_t i;

    if (jy901 == NULL)
    {
        return 0;
    }

    if (received_length == 0 && data != JY901_FRAME_HEADER)
    {
        return 0;
    }

    frame[received_length] = data;
    received_length++;

    if (received_length < JY901_FRAME_LENGTH)
    {
        return 0;
    }

    received_length = 0;

    for (i = 0; i < JY901_CHECKSUM_INDEX; i++)
    {
        checksum = (uint8_t)(checksum + frame[i]);
    }

    if (checksum != frame[JY901_CHECKSUM_INDEX])
    {
        return 0;
    }

    jy901_parse_frame(frame, jy901);
    return frame[1] == JY901_PACKET_ANGLE;
}

/**
 * @brief 按照项目预设参数打开JY901串口。
 *
 * 当前依赖JY901已经保存为9600波特率并开启主动输出。本函数只打开串口，
 * 不恢复出厂设置，也不修改或保存传感器内部配置。
 *
 * @return 成功返回串口文件描述符，失败返回-1。
 */
int jy901Setup(void)
{
    return serialOpen(JY901_UART_DEVICE, JY901_UART_BAUD);
}
