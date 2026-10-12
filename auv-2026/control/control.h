/**
 * @file control.h
 * @brief ROV控制算法的常量、状态类型和公共接口声明。
 *
 * 本头文件定义输入范围及推进器混控相关接口，连接设备
 * 控制线程与具体控制算法。修改控制方向或闭环策略时应同步检查这些定义。
 */

#ifndef __CONTROL_H_
#define __CONTROL_H_

#include "../user/config.h"

#include "../applications/pwmDevices.h"

/* 转换后的摇杆控制量。 */
typedef struct
{
    int16_t fx;    // X轴：左右平移，正值向右
    int16_t fy;    // Y轴：前进后退，正值向前
    int16_t fz;    // Z轴：上浮下潜，正值上浮
    int16_t yaw;   // 偏航：绕竖直轴自转，正值向右
} rockerInfo_t;

/* 当前ROV实际使用的手柄控制链。 */
void get_rocker_params(rockerInfo_t *rc);
void rov_horizontal_control(rockerInfo_t *rc);
void rov_vertical_control(rockerInfo_t *rc);
void rov_propeller_output(propellerPower_t *propeller);

#endif
