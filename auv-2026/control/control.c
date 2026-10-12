/**
 * @file control.c
 * @brief ROV手柄控制和六推进器混控。
 *
 * 当前比赛ROV采用开环手柄控制：上位机摇杆数据依次经过水平/垂直混控、
 * 单推进器参数修正和死区补偿，最后交给PWM设备层。
 */

#include <stdlib.h>

#include "../applications/data.h"
#include "../connections/rovCommandState.h"
#include "control.h"

#define ROCKER_DEADBAND            10
#define HORIZONTAL_ROCKER_DEADBAND 30
static propellerPower_t propeller_buffer = {0}; /* 六路基础控制量。 */

/**
 * @brief 读取上位机摇杆数据并转换为控制输入。
 * @param rc 输出的摇杆控制量。
 */
void get_rocker_params(rockerInfo_t *rc)
{
    rov_command_state_t state;

    rov_command_state_get_snapshot(&state);
    rc->fx  = (int16_t)state.move.x;
    rc->fy  = (int16_t)state.move.y;
    rc->fz  = (int16_t)state.move.z;
    rc->yaw = (int16_t)state.move.yaw;

    if (abs(rc->fz) < ROCKER_DEADBAND)
    {
        rc->fz = 0;
    }
}

/**
 * @brief 根据Z轴摇杆直接控制两台垂直推进器。
 * @param rc 本周期摇杆输入。
 *
 * 本函数只处理手柄死区、推进器方向、使能及正反向动力比例，不读取定深
 * 开关、AUV任务或PID状态，是当前比赛ROV使用的垂直控制入口。
 */
void rov_vertical_control(rockerInfo_t *rc)
{
    rov_command_state_t state;

    rov_command_state_get_snapshot(&state);
    if (rc->fz > ROCKER_DEADBAND)
    {
        propeller_buffer.leftMiddle = rc->fz *
                                     state.propellers.center_left.direction_multiplier *
                                     state.propellers.center_left.enabled *
                                     state.propellers.center_left.power_positive;
        propeller_buffer.rightMiddle = rc->fz *
                                      state.propellers.center_right.direction_multiplier *
                                      state.propellers.center_right.enabled *
                                      state.propellers.center_right.power_positive;
    }
    else if (rc->fz < -ROCKER_DEADBAND)
    {
        propeller_buffer.leftMiddle = rc->fz *
                                     state.propellers.center_left.direction_multiplier *
                                     state.propellers.center_left.enabled *
                                     state.propellers.center_left.power_negative;
        propeller_buffer.rightMiddle = rc->fz *
                                      state.propellers.center_right.direction_multiplier *
                                      state.propellers.center_right.enabled *
                                      state.propellers.center_right.power_negative;
    }
    else
    {
        propeller_buffer.leftMiddle  = 0;
        propeller_buffer.rightMiddle = 0;
    }
}

/**
 * @brief 将单台水平推进器的混控需求转换为校准后的基础控制量。
 * @param demand 带符号的混控需求。
 * @param config 该推进器的方向、使能和正反向动力参数。
 */
static int16_t horizontal_propeller_output(
    float demand,
    const propeller_runtime_config_t *config)
{
    const double power_ratio =
        demand >= 0 ? config->power_positive : config->power_negative;

    return (int16_t)(demand * config->direction_multiplier *
                     config->enabled * power_ratio);
}

/**
 * @brief 根据标准六推进器布局计算四台水平推进器的基础控制量。
 * @param rc 本周期摇杆输入。
 *
 * 坐标约定：+Y前进，+X右平移，+Yaw为俯视顺时针右自转。推进器编号为
 * 1=rightUp、2=leftUp、3=rightDown、4=leftDown，混控矩阵如下：
 *
 *     1 = +Y - X - Yaw       2 = +Y + X + Yaw
 *     3 = +Y + X - Yaw       4 = +Y - X + Yaw
 *
 * 本函数只使用摇杆输入，不叠加航向PID。reversed用于修正安装或接线方向，
 * power_positive/power_negative在混控完成后按最终控制量的正负选择。
 */
void rov_horizontal_control(rockerInfo_t *rc)
{
    rov_command_state_t state;
    rov_command_state_get_snapshot(&state);

    const int16_t horizontal_x =
        abs(rc->fx) > HORIZONTAL_ROCKER_DEADBAND ? rc->fx : 0;
    const int16_t horizontal_y =
        abs(rc->fy) > HORIZONTAL_ROCKER_DEADBAND ? rc->fy : 0;
    const int16_t manual_yaw =
        abs(rc->yaw) > HORIZONTAL_ROCKER_DEADBAND ? rc->yaw : 0;

    const float demand_1 = +horizontal_y - horizontal_x - manual_yaw;
    const float demand_2 = +horizontal_y + horizontal_x + manual_yaw;
    const float demand_3 = +horizontal_y + horizontal_x - manual_yaw;
    const float demand_4 = +horizontal_y - horizontal_x + manual_yaw;

    propeller_buffer.rightUp = horizontal_propeller_output(
        demand_1, &state.propellers.front_right);

    propeller_buffer.leftUp = horizontal_propeller_output(
        demand_2, &state.propellers.front_left);

    propeller_buffer.rightDown = horizontal_propeller_output(
        demand_3, &state.propellers.back_right);
        
    propeller_buffer.leftDown = horizontal_propeller_output(
        demand_4, &state.propellers.back_left);
}

/**
 * @brief 为六路基础控制量加入对应方向的死区补偿。
 * @param propeller 输出给PWM设备层的六路最终控制量。
 *
 * 基础控制量大于5时加入deadzone_upper，小于-5时加入deadzone_lower；停转
 * 附近不补偿，避免推进器因补偿量意外启动。
 */
void rov_propeller_output(propellerPower_t *propeller)
{
    rov_command_state_t state;
    rov_command_state_get_snapshot(&state);

    propeller->leftDown = propeller_buffer.leftDown +
        (propeller_buffer.leftDown > 5 ? state.propellers.back_left.deadzone_upper :
         propeller_buffer.leftDown < -5 ? state.propellers.back_left.deadzone_lower : 0);
    propeller->rightDown = propeller_buffer.rightDown +
        (propeller_buffer.rightDown > 5 ? state.propellers.back_right.deadzone_upper :
         propeller_buffer.rightDown < -5 ? state.propellers.back_right.deadzone_lower : 0);
    propeller->leftUp = propeller_buffer.leftUp +
        (propeller_buffer.leftUp > 5 ? state.propellers.front_left.deadzone_upper :
         propeller_buffer.leftUp < -5 ? state.propellers.front_left.deadzone_lower : 0);
    propeller->rightUp = propeller_buffer.rightUp +
        (propeller_buffer.rightUp > 5 ? state.propellers.front_right.deadzone_upper :
         propeller_buffer.rightUp < -5 ? state.propellers.front_right.deadzone_lower : 0);
    propeller->leftMiddle = propeller_buffer.leftMiddle +
        (propeller_buffer.leftMiddle > 5 ? state.propellers.center_left.deadzone_upper :
         propeller_buffer.leftMiddle < -5 ? state.propellers.center_left.deadzone_lower : 0);
    propeller->rightMiddle = propeller_buffer.rightMiddle +
        (propeller_buffer.rightMiddle > 5 ? state.propellers.center_right.deadzone_upper :
         propeller_buffer.rightMiddle < -5 ? state.propellers.center_right.deadzone_lower : 0);
}
