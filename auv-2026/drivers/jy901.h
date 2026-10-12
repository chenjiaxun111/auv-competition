/**
 * @file jy901.h
 * @brief JY901公开数据类型和驱动接口。
 */

#ifndef JY901_H
#define JY901_H

#include <stdint.h>

/** 三轴浮点数据。 */
typedef struct
{
    float x;
    float y;
    float z;
} jy901_vector3f_t;

/** 三轴有符号16位整数数据。 */
typedef struct
{
    int16_t x;
    int16_t y;
    int16_t z;
} jy901_vector3i16_t;

/**
 * JY901解析结果。
 *
 * @warning 当前ROV铁质舱体及内部电气设备会干扰磁力计，姿态数据尚未完成
 * 整机验证，不得直接用于航向闭环控制。
 */
typedef struct
{
    jy901_vector3f_t acc;   // 三轴加速度，单位g
    jy901_vector3f_t gyro;  // 三轴角速度，单位°/s
    jy901_vector3i16_t mag; // 三轴磁场原始值，单位LSB
    float pressure;         // 大气压力，单位Pa，仅JY901B有效
    float altitude;         // 气压高度，单位m，仅JY901B有效
    float roll;             // 横滚角，单位°
    float pitch;            // 俯仰角，单位°
    float yaw;              // 航向角，单位°
    float temperature;      // 模块温度，单位℃
} jy901_t;

/** 按照项目预设参数打开JY901串口。 */
int jy901Setup(void);

/** 接收一个串口字节，组装并解析JY901标准11字节数据包。 */
/** @return 完整有效角度包返回1，其余返回0。 */
int copeJY901_data(uint8_t data, jy901_t *jy901);

#endif
