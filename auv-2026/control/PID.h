/** @file PID.h @brief 固定周期位置式PID控制器。 */
#ifndef AUV_PID_H
#define AUV_PID_H

#include <stdint.h>

typedef struct {
    uint16_t p, i, d;
    uint16_t output_limit, integral_limit, integral_separation_error;
    uint8_t p_enabled, i_enabled, d_enabled;
    uint8_t integral_limit_enabled, integral_separation_enabled;
} pid_config_t;

typedef struct {
    float target, feedback, error, integral;
    float p_output, i_output, d_output, output;
} pid_runtime_t;

typedef struct {
    pid_config_t config;
    pid_runtime_t runtime;
    float effective_kp, effective_ki_dt, effective_kd_over_dt;
    float last_feedback;
    uint8_t feedback_initialized;
} PID_Controller;

typedef struct {
    PID_Controller Roll_Gyro_Control, Pitch_Gyro_Control, Yaw_Gyro_Control;
    PID_Controller Roll_Angle_Control, Pitch_Angle_Control, Yaw_Angle_Control;
    PID_Controller High_Speed_Control, High_Position_Control;
    PID_Controller Location_X_Control, Location_Y_Control;
} AllController;

AllController *pid_controllers_get(void);
int pid_controller_configure(PID_Controller *controller, const pid_config_t *config);
int pid_controller_get_config(const PID_Controller *controller, pid_config_t *config);
void pid_controller_set_target(PID_Controller *controller, float target);
void pid_controller_reset(PID_Controller *controller);
float pid_controller_update(PID_Controller *controller, float feedback, int wrap_angle);
int pid_controller_get_runtime(const PID_Controller *controller, pid_runtime_t *runtime);

#endif
