/**
 * @file config.h
 * @brief AUV/ROV整机功能、硬件连接和公共运行参数配置。
 */

#ifndef CONFIG_H
#define CONFIG_H

#include <stdint.h>

#define AUV_MASTER_VERSION "2.0.0"  // 程序版本

/* ----------------------【航行器定义】---------------------------------*/
/* 编译前在这里选择航行器型号。ROV保持原开环功能，AUV启用闭环。 */
#define VEHICLE_MODEL_ROV 0
#define VEHICLE_MODEL_AUV 1
#ifndef VEHICLE_MODEL
#define VEHICLE_MODEL     VEHICLE_MODEL_AUV
#endif

#if VEHICLE_MODEL != VEHICLE_MODEL_ROV && VEHICLE_MODEL != VEHICLE_MODEL_AUV
#error "VEHICLE_MODEL must be VEHICLE_MODEL_ROV or VEHICLE_MODEL_AUV"
#endif

/* ----------------------【设备功能开关】-------------------------------*/
/* 所有设备启用开关集中放在配置文件，业务模块只读取，不重复定义。 */
#define PROPELLER_ENABLED 1  // 推进器控制和PWM输出
#define ROBOT_ARM_ENABLED 1  // 机械臂控制和PWM输出

/* ----------------------【传感器配置】---------------------------------*/

/* ADS1118电源采集尚未完成硬件标定，完成前保持关闭。 */
#define ADS1118_ENABLED 0
#define ADS1118_PIN_BASE 500

/* AUV航向环必须启用JY901；ROV保持原来默认关闭的行为。 */
#define JY901_ENABLED     (VEHICLE_MODEL == VEHICLE_MODEL_AUV)
#define JY901_UART_DEVICE "/dev/ttyS2"
#define JY901_UART_BAUD   9600

/* 深度传感器类型及硬件连接。 */
#define MS5837_IIC  0 // MS5837直接连接I2C
#define MS5837_UART 1 // MS5837解算板串口输出

#define DEPTHSENSOR          MS5837_UART
#define DEPTH_SENSOR_PIN_BASE 360

#define MS5837_I2C_DEVICE  "/dev/i2c-2"
#define MS5837_UART_DEVICE "/dev/ttyS1"
#define MS5837_UART_BAUD   115200

/* ----------------------【AUV闭环控制】-------------------------------*/
/* 传感器与推进器方向未经实机验证，低功率测试后只需修改对应正负号。 */
#define AUV_DEPTH_FEEDBACK_DIRECTION  (-1.0f)
#define AUV_YAW_FEEDBACK_DIRECTION     (1.0f)
#define AUV_DEPTH_OUTPUT_DIRECTION     (1.0f)
#define AUV_YAW_OUTPUT_DIRECTION       (1.0f)

#define AUV_CONTROL_PERIOD_MS          20U
#define AUV_SENSOR_TIMEOUT_MS          500U
#define AUV_SENSOR_RECOVERY_FRAMES     3U
#define AUV_HEARTBEAT_TIMEOUT_MS       3000U

#define AUV_DEPTH_TARGET_MIN_M         (-2.0f)
#define AUV_DEPTH_TARGET_MAX_M         (0.0f)
#define AUV_RELATIVE_DEPTH_MIN_M       (-2.0f)
#define AUV_RELATIVE_DEPTH_MAX_M       (2.0f)
#define AUV_RELATIVE_YAW_MIN_DEG       (-180.0f)
#define AUV_RELATIVE_YAW_MAX_DEG       (180.0f)

#define AUV_DEPTH_MANUAL_DEADBAND      10
#define AUV_YAW_MANUAL_DEADBAND        20

/* 上位机用整数传输增益和积分分离阈值；运行值为传输值除以10。 */
#define PID_PARAMETER_SCALE            10.0f
#define PID_GAIN_SETTING_MAX           200U
#define PID_OUTPUT_LIMIT_MAX           500U
#define PID_INTEGRAL_LIMIT_MAX         500U
#define PID_DEPTH_SEPARATION_MAX       100U
#define PID_YAW_SEPARATION_MAX         1800U

/* ----------------------【PCA9685硬件连接】----------------------------*/
/* NanoPi与PCA9685的基础连接。OE为低电平使能，使用WiringPi编号。 */
#define PCA9685_I2C_DEVICE  "/dev/i2c-1"
#define PCA9685_I2C_ADDRESS 0x40
#define PCA9685_OE_PIN      19
#define PCA9685_PIN_BASE    300
#define PCA9685_OSC_CLK     25000000.0f

/*
 * PCA9685输出通道编号范围为0～15。
 * 六路推进器可以按实际接线分别配置；修改通道不需要再改业务代码。
 */
#define PROPELLER_RIGHT_UP_CHANNEL     5
#define PROPELLER_RIGHT_MIDDLE_CHANNEL 4
#define PROPELLER_RIGHT_DOWN_CHANNEL   3
#define PROPELLER_LEFT_UP_CHANNEL      2
#define PROPELLER_LEFT_MIDDLE_CHANNEL  6
#define PROPELLER_LEFT_DOWN_CHANNEL    7
#define ROBOT_ARM_CHANNEL 1

/* ------------------------【数据定义】---------------------------------*/
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt))) //限幅函数

#endif
