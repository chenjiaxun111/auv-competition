/**
 * @desc: 各个传感器线程 (深度传感器、九轴、CPU 设备数据解析并获取)
 */

#define LOG_TAG "sensor"

#include <elog.h>
#include <pthread.h>
#include <time.h>
#include <wiringPi.h>
#include <wiringSerial.h>

#include "../drivers/ads1118.h"
#include "../drivers/jy901.h"
#include "../drivers/ms5837.h"
#include "../user/config.h"
#include "sensor.h"

#define ADS1118_VOLTAGE_PIN    ADS1118_PIN_BASE
#define ADS1118_CURRENT_PIN    (ADS1118_PIN_BASE + 1)
#define POWER_SAMPLE_PERIOD_MS 1000

static int jy901_fd;
static int depth_fd;
static pthread_mutex_t sensor_state_mutex = PTHREAD_MUTEX_INITIALIZER;
static depthSensor_t depth_sensor_state;
static powerSource_t power_source_state;
static jy901_t jy901_state;
static sensor_health_t depth_health;
static sensor_health_t heading_health;
static int depth_zero_ready;
static float depth_zero;

static uint64_t monotonic_ms(void)
{
    struct timespec now;
    if (clock_gettime(CLOCK_MONOTONIC, &now) != 0) return 0;
    return (uint64_t)now.tv_sec * 1000U + (uint64_t)now.tv_nsec / 1000000U;
}

int sensor_get_depth_health(sensor_health_t *snapshot)
{
    if (!snapshot) return -1;
    pthread_mutex_lock(&sensor_state_mutex); *snapshot = depth_health;
    pthread_mutex_unlock(&sensor_state_mutex); return 0;
}

int sensor_get_heading_health(sensor_health_t *snapshot)
{
    if (!snapshot) return -1;
    pthread_mutex_lock(&sensor_state_mutex); *snapshot = heading_health;
    pthread_mutex_unlock(&sensor_state_mutex); return 0;
}

int sensor_get_depth(depthSensor_t *snapshot)
{
    if (snapshot == NULL) return -1;
    pthread_mutex_lock(&sensor_state_mutex);
    *snapshot = depth_sensor_state;
    pthread_mutex_unlock(&sensor_state_mutex);
    return 0;
}

int sensor_get_power(powerSource_t *snapshot)
{
    if (snapshot == NULL) return -1;
    pthread_mutex_lock(&sensor_state_mutex);
    *snapshot = power_source_state;
    pthread_mutex_unlock(&sensor_state_mutex);
    return 0;
}

int sensor_get_jy901(jy901_t *snapshot)
{
    if (snapshot == NULL) return -1;
    pthread_mutex_lock(&sensor_state_mutex);
    *snapshot = jy901_state;
    pthread_mutex_unlock(&sensor_state_mutex);
    return 0;
}

/**
 * @brief ADS1118电源采集线程（硬件标定未完成）。
 *
 * 当前只完成：AIN0/AIN1原始值读取，以及按照ADS1118量程换算输入引脚电压。
 *
 * TODO：
 * 1. 核对AIN0、AIN1与电压、电流采样电路的实际接线。
 * 2. 根据分压电阻计算电池电压还原系数，并用万用表进行多点校准。
 * 3. 空载测量电流传感器零点电压。
 * 4. 使用已知负载和电流表测量电流传感器实际灵敏度，单位V/A。
 * 5. 确认电流正负方向，并将AIN1电压换算为真实电流，单位A。
 * 6. 检查最大电池电压和最大传感器输出不会超过ADS1118输入范围。
 * 7. 在推进器启停和堵转工况下测试噪声，再决定滤波与采样周期。
 * 8. 完成以上测试后，才允许添加欠压、过流和功率保护逻辑。
 */
static void *adc_thread(void *arg)
{
    int16_t voltage_raw;
    int16_t current_raw;

    (void)arg;

    while (1)
    {
        voltage_raw = (int16_t)analogRead(ADS1118_VOLTAGE_PIN);
        current_raw = (int16_t)analogRead(ADS1118_CURRENT_PIN);

        /* AIN0换算结果仍是ADC引脚电压，不是经过分压还原后的电池电压。 */
        pthread_mutex_lock(&sensor_state_mutex);
        power_source_state.voltage = ads1118RawToVoltage(voltage_raw);

        /* AIN1换算结果仍是传感器输出电压，不是经过标定后的电流安培值。 */
        power_source_state.current = ads1118RawToVoltage(current_raw);
        pthread_mutex_unlock(&sensor_state_mutex);

        delay(POWER_SAMPLE_PERIOD_MS);
    }

    return NULL;
}

/**
 * @brief 持续接收并解析JY901主动输出的串口数据。
 * @param arg 指向JY901串口文件描述符的指针。
 * @return 本线程正常运行时不会退出，返回值仅满足pthread线程函数接口。
 *
 * @note 当前JY901由JY901_ENABLED控制，默认不在ROV中启动。启用前应确认
 * 模块已设置为9600波特率、主动输出模式，并输出本驱动需要的数据包。
 *
 * JY901支持通过配置寄存器设置加速度、角速度和磁场三轴零偏，也可以设置
 * 回传内容、回传频率、安装方向及姿态算法。建议使用厂商工具完成实机校准，
 * 或严格按照对应固件手册执行“解锁、写寄存器、保存”流程；不能在本接收
 * 线程中反复写入配置，也不能在每次启动时恢复出厂设置，否则会清除校准值。
 *
 * @note JY901系列存在不同硬件批次和固件版本。旧版资料可能允许直接写
 * 配置寄存器，新版协议通常要求先发送解锁命令，部分寄存器及配置流程也
 * 可能变化。修改模块配置前必须确认具体型号、固件版本及对应版本手册，
 * 不要直接套用项目中的旧命令或其他JY901/WT901型号的说明。
 *
 * @warning 铁质舱体、推进器、电调和大电流电源线会干扰JY901磁力计。
 * 即使添加磁场零偏，也必须经过整机带载验证，当前数据不得直接用于ROV
 * 航向闭环控制。
 */
static void *jy901_thread(void *arg)
{
    uint8_t data;
    int fd = *(int *)arg;

    while (1)
    {
        delay(10);

        while (serialDataAvail(fd) > 0)
        {
            data = (uint8_t)serialGetchar(fd);
            pthread_mutex_lock(&sensor_state_mutex);
            if (copeJY901_data(data, &jy901_state)) {
                heading_health.last_update_ms = monotonic_ms();
                heading_health.sample_count++;
            }
            pthread_mutex_unlock(&sensor_state_mutex);
        }
    }

    return NULL;
}

/**
 * @brief 接收并处理MS5837解算板的串口数据。
 */
static void *ms5837_usart_thread(void *arg)
{
    uint8_t data;
    float depth;
    float temperature;
    int fd = *(int *)arg;

    while (1)
    {
        delay(10);

        while (serialDataAvail(fd) > 0)
        {
            data = (uint8_t)serialGetchar(fd);

            if (ms5837_usart_data(data, &temperature, &depth) == 1)
            {
                /* 串口解算板的深度数据直接使用，暂不进行滤波。 */
                pthread_mutex_lock(&sensor_state_mutex);
#if VEHICLE_MODEL == VEHICLE_MODEL_AUV
                if (!depth_zero_ready) { depth_zero = depth; depth_zero_ready = 1; }
                depth_sensor_state.depth = depth - depth_zero;
                depth_sensor_state.depth *= AUV_DEPTH_FEEDBACK_DIRECTION;
#else
                depth_sensor_state.depth = depth;
#endif
                depth_sensor_state.temperature = temperature;
                depth_health.last_update_ms = monotonic_ms();
                depth_health.sample_count++;
                pthread_mutex_unlock(&sensor_state_mutex);
            }
        }
    }

    return NULL;
}

/**
 * @brief 读取并处理MS5837的I2C直连数据。
 */
static void *ms5837_iic_thread(void *arg)
{
    float pressure;
    float surface_pressure = 0.0f;
    float temperature;
    int fd = *(int *)arg;
    int baseline_ready = 0;

    while (1)
    {
        if (ms5837_IIC_data(fd, &temperature, &pressure) == 1)
        {
            if (!baseline_ready)
            {
                /* 第一组有效压力作为启动时的零深度基准。 */
                surface_pressure = pressure;
                baseline_ready = 1;
            }

            /* p=ρgh：现有项目按淡水密度1000 kg/m³换算深度。 */
            pthread_mutex_lock(&sensor_state_mutex);
            depth_sensor_state.depth = (pressure - surface_pressure) /
                (1000.0f * 9.8f);
#if VEHICLE_MODEL == VEHICLE_MODEL_AUV
            depth_sensor_state.depth *= AUV_DEPTH_FEEDBACK_DIRECTION;
#endif
            depth_sensor_state.temperature = temperature;
            depth_health.last_update_ms = monotonic_ms();
            depth_health.sample_count++;
            pthread_mutex_unlock(&sensor_state_mutex);
        }

        delay(100);
    }

    return NULL;
}

/**
 * @brief  传感器线程初始化
 */
int sensor_thread_init(void)
{
    pthread_t adc_tid;
    pthread_t depth_tid;
    pthread_t jy901_tid;
    int adc_fd;

    if (ADS1118_ENABLED)
    {
        /* ADS1118硬件标定完成并开启宏后，才初始化设备和创建采集线程。 */
        adc_fd = ads1118Setup(ADS1118_PIN_BASE);
        if (adc_fd < 0)
        {
            ERROR_LOG(adc_fd, "ADS1118");
        }
        else
        {
            log_i("ADS1118 init");
            pthread_create(&adc_tid, NULL, adc_thread, NULL);
            pthread_detach(adc_tid);
        }
    }

    if (JY901_ENABLED)
    {
        jy901_fd = jy901Setup();
        if (jy901_fd < 0)
        {
            ERROR_LOG(jy901_fd, "JY901");
        }
        else
        {
            log_i("JY901 init");
            pthread_create(&jy901_tid, NULL, jy901_thread, &jy901_fd);
            pthread_detach(jy901_tid);
        }
    }

    if (DEPTHSENSOR == MS5837_UART)
    {
        depth_fd = ms5837_usart_setup();
        if (depth_fd < 0)
        {
            ERROR_LOG(depth_fd, "MS5837");
        }
        else
        {
            log_i("MS5837 UART init");
            pthread_create(&depth_tid, NULL, ms5837_usart_thread, &depth_fd);
            pthread_detach(depth_tid);
        }
    }
    else if (DEPTHSENSOR == MS5837_IIC)
    {
        depth_fd = ms5837_IIC_setup(DEPTH_SENSOR_PIN_BASE);

        if (depth_fd < 0)
        {
            ERROR_LOG(depth_fd, "MS5837");
        }
        else
        {
            log_i("MS5837 I2C init");
            pthread_create(&depth_tid, NULL, ms5837_iic_thread, &depth_fd);
            pthread_detach(depth_tid);
        }
    }
    return 0;
}
