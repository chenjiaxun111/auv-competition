/** @file PID.c @brief 固定周期位置式PID实现。 */
#include <math.h>
#include <pthread.h>
#include <string.h>
#include "PID.h"
#include "../user/config.h"

static AllController pid_controllers;
static pthread_mutex_t pid_mutex = PTHREAD_MUTEX_INITIALIZER;

static float limit(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
static float wrap_degrees(float v)
{
    while (v >= 180.0f) v -= 360.0f;
    while (v < -180.0f) v += 360.0f;
    return v;
}

AllController *pid_controllers_get(void) { return &pid_controllers; }

int pid_controller_configure(PID_Controller *c, const pid_config_t *cfg)
{
    const float dt = AUV_CONTROL_PERIOD_MS / 1000.0f;
    float target;
    if (!c || !cfg || dt <= 0.0f) return -1;
    pthread_mutex_lock(&pid_mutex);
    target = c->runtime.target;
    c->config = *cfg;
    c->effective_kp = cfg->p_enabled ? cfg->p / PID_PARAMETER_SCALE : 0.0f;
    c->effective_ki_dt = cfg->i_enabled ? (cfg->i / PID_PARAMETER_SCALE) * dt : 0.0f;
    c->effective_kd_over_dt = cfg->d_enabled ? (cfg->d / PID_PARAMETER_SCALE) / dt : 0.0f;
    memset(&c->runtime, 0, sizeof(c->runtime));
    c->runtime.target = target;
    c->last_feedback = 0.0f;
    c->feedback_initialized = 0;
    pthread_mutex_unlock(&pid_mutex);
    return 0;
}

int pid_controller_get_config(const PID_Controller *c, pid_config_t *cfg)
{
    if (!c || !cfg) return -1;
    pthread_mutex_lock(&pid_mutex);
    *cfg = c->config;
    pthread_mutex_unlock(&pid_mutex);
    return 0;
}

void pid_controller_set_target(PID_Controller *c, float target)
{
    if (!c) return;
    pthread_mutex_lock(&pid_mutex);
    memset(&c->runtime, 0, sizeof(c->runtime));
    c->runtime.target = target;
    c->last_feedback = 0.0f;
    c->feedback_initialized = 0;
    pthread_mutex_unlock(&pid_mutex);
}

void pid_controller_reset(PID_Controller *c)
{
    if (!c) return;
    pthread_mutex_lock(&pid_mutex);
    memset(&c->runtime, 0, sizeof(c->runtime));
    c->last_feedback = 0.0f;
    c->feedback_initialized = 0;
    pthread_mutex_unlock(&pid_mutex);
}

float pid_controller_update(PID_Controller *c, float feedback, int wrap_angle)
{
    float derivative = 0.0f, separation, integral_limit, output_limit, result;
    if (!c || !isfinite(feedback)) return 0.0f;
    pthread_mutex_lock(&pid_mutex);
    c->runtime.feedback = feedback;
    c->runtime.error = c->runtime.target - feedback;
    if (wrap_angle) c->runtime.error = wrap_degrees(c->runtime.error);
    separation = c->config.integral_separation_error / PID_PARAMETER_SCALE;
    if (c->config.i_enabled && (!c->config.integral_separation_enabled ||
        fabsf(c->runtime.error) <= separation))
        c->runtime.integral += c->effective_ki_dt * c->runtime.error;
    if (c->config.integral_limit_enabled) {
        integral_limit = (float)c->config.integral_limit;
        c->runtime.integral = limit(c->runtime.integral, -integral_limit, integral_limit);
    }
    if (c->feedback_initialized) derivative = -(feedback - c->last_feedback);
    c->last_feedback = feedback;
    c->feedback_initialized = 1;
    c->runtime.p_output = c->effective_kp * c->runtime.error;
    c->runtime.i_output = c->runtime.integral;
    c->runtime.d_output = c->effective_kd_over_dt * derivative;
    result = c->runtime.p_output + c->runtime.i_output + c->runtime.d_output;
    output_limit = (float)c->config.output_limit;
    c->runtime.output = limit(result, -output_limit, output_limit);
    result = c->runtime.output;
    pthread_mutex_unlock(&pid_mutex);
    return result;
}

int pid_controller_get_runtime(const PID_Controller *c, pid_runtime_t *runtime)
{
    if (!c || !runtime) return -1;
    pthread_mutex_lock(&pid_mutex);
    *runtime = c->runtime;
    pthread_mutex_unlock(&pid_mutex);
    return 0;
}
