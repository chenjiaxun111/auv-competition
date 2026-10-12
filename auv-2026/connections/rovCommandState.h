/**
 * @file rovCommandState.h
 * @brief 上位机控制命令及推进器运行配置的线程安全共享状态。
 */

#ifndef ROV_COMMAND_STATE_H
#define ROV_COMMAND_STATE_H

#include <stdint.h>

/** @brief 单台推进器在控制计算阶段使用的运行参数。 */
typedef struct
{
    int deadzone_upper;          /**< 正向输出的死区补偿量。 */
    int deadzone_lower;          /**< 反向输出的死区补偿量。 */
    double power_positive;       /**< 正向动力比例。 */
    double power_negative;       /**< 反向动力比例。 */
    int direction_multiplier;    /**< 1为正常方向，-1为反向。 */
    int enabled;                 /**< 1为启用推进器，0为禁止输出。 */
} propeller_runtime_config_t;

/** @brief 六路推进器运行参数，名称与参数JSON中的键名一致。 */
typedef struct
{
    propeller_runtime_config_t back_left;
    propeller_runtime_config_t back_right;
    propeller_runtime_config_t center_left;
    propeller_runtime_config_t center_right;
    propeller_runtime_config_t front_left;
    propeller_runtime_config_t front_right;
} propeller_runtime_configs_t;

/** @brief 上位机运动指令转换后的四自由度控制量。 */
typedef struct
{
    double x;      /**< 左右平移控制量。 */
    double y;      /**< 前后运动控制量。 */
    double z;      /**< 上浮、下潜控制量。 */
    double yaw;    /**< 偏航旋转控制量。 */
} rov_move_command_t;

/**
 * @brief RPC接收线程与ROV控制线程之间传递的完整状态。
 *
 * 该结构体的内部实例由实现文件统一加锁管理。外部代码应通过本文件提供的
 * 函数读写，不应保存或直接修改内部实例地址。
 */
typedef struct
{
    rov_move_command_t move;                 /**< 最近一次运动指令。 */
    propeller_runtime_configs_t propellers;  /**< 六路推进器运行参数。 */
    float target_depth;                      /**< 一次性相对深度。 */
    float target_heading;                    /**< 一次性相对航向。 */
    int depth_target_pending;                /**< 目标深度是否尚未取走。 */
    int heading_target_pending;              /**< 目标航向是否尚未取走。 */
    float gripper_input;                     /**< 机械爪控制输入。 */
} rov_command_state_t;

/**
 * @brief 线程安全地复制完整控制状态。
 * @param snapshot 接收状态副本，不得为NULL。
 * @return 成功返回0，参数无效返回-1。
 */
int rov_command_state_get_snapshot(rov_command_state_t *snapshot);

/** @brief 原子地更新四自由度运动控制量。 */
void rov_command_state_set_move(double x, double y, double z, double yaw);

/**
 * @brief 超过指定时间未收到运动指令时，将四自由度控制量自动清零。
 * @param timeout_ms 运动指令有效期，单位毫秒。
 * @return 本次触发超时并清零返回1，未超时返回0，时钟读取失败返回-1。
 */
void rov_command_state_note_heartbeat(void);
int rov_command_state_heartbeat_alive(unsigned int timeout_ms);

/** @brief 更新机械爪控制输入。 */
void rov_command_state_set_gripper(float input);

/**
 * @brief 写入一次性深度和航向目标，并将两个目标标记为待处理。
 * @note 目标值可以为0，是否存在新目标由独立pending标志判断。
 */
void rov_command_state_set_auv_target(float depth, int has_depth,
                                      float heading, int has_heading);

/**
 * @brief 取出目标深度并清除其待处理标志。
 * @return 有新目标返回1，无新目标返回0，参数无效返回-1。
 */
int rov_command_state_take_depth_target(float *target);

/**
 * @brief 取出目标航向并清除其待处理标志。
 * @return 有新目标返回1，无新目标返回0，参数无效返回-1。
 */
int rov_command_state_take_heading_target(float *target);

/**
 * @brief 一次性替换全部六路推进器运行参数。
 * @param propellers 已完成合法性校验的参数，不得为NULL。
 */
void rov_command_state_update_propellers(
    const propeller_runtime_configs_t *propellers);

#endif /* ROV_COMMAND_STATE_H */
