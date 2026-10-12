/**
 * @file data.h
 * @brief parameters目录中各类持久化参数的加载与保存接口。
 */

#ifndef ROV_DATA_H
#define ROV_DATA_H

/* 将当前PID和普通PWM设备参数保存到parameters/control_parameters.json。 */
int pid_and_pwm_device_parameters_save(void);

/* 从parameters/control_parameters.json加载参数；成功返回0，失败返回-1。 */
int pid_and_pwm_device_parameters_load(void);

/* 从parameters/propeller_parameters.json加载推进器参数。 */
int propeller_parameters_load(void);

/* 将当前推进器参数保存到parameters/propeller_parameters.json。 */
int propeller_parameters_save(void);

#endif /* ROV_DATA_H */
