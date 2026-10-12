/**
 * @file rovCommandState.c
 * @brief ROV控制命令共享状态的线程安全实现。
 *
 * JSON-RPC线程负责写入命令，控制与PWM线程负责读取。所有共享状态均由
 * state_mutex保护；读取方获得的是结构体副本，解锁后不会继续访问内部数据。
 */

#include <pthread.h>
#include <time.h>

#include "rovCommandState.h"

static rov_command_state_t state; /**< 进程内唯一的控制命令状态。 */
static pthread_mutex_t state_mutex = PTHREAD_MUTEX_INITIALIZER; /**< 状态锁。 */
static struct timespec last_heartbeat_time;
static int heartbeat_received;

/** @copydoc rov_command_state_get_snapshot */
int rov_command_state_get_snapshot(rov_command_state_t *snapshot)
{
    if (snapshot == NULL) return -1;
    pthread_mutex_lock(&state_mutex);
    *snapshot = state;
    pthread_mutex_unlock(&state_mutex);
    return 0;
}

/** @copydoc rov_command_state_set_move */
void rov_command_state_set_move(double x, double y, double z, double yaw)
{
    pthread_mutex_lock(&state_mutex);
    state.move.x = x; state.move.y = y; state.move.z = z; state.move.yaw = yaw;
    pthread_mutex_unlock(&state_mutex);
}

/** @copydoc rov_command_state_note_heartbeat */
void rov_command_state_note_heartbeat(void)
{
    pthread_mutex_lock(&state_mutex);
    if (clock_gettime(CLOCK_MONOTONIC, &last_heartbeat_time) == 0)
        heartbeat_received = 1;
    pthread_mutex_unlock(&state_mutex);
}

int rov_command_state_heartbeat_alive(unsigned int timeout_ms)
{
    struct timespec now;
    int64_t elapsed_ms;
    int alive = 0;
    pthread_mutex_lock(&state_mutex);
    if (heartbeat_received && clock_gettime(CLOCK_MONOTONIC, &now) == 0) {
        elapsed_ms = (int64_t)(now.tv_sec - last_heartbeat_time.tv_sec) * 1000
                   + (int64_t)(now.tv_nsec - last_heartbeat_time.tv_nsec) / 1000000;
        alive = elapsed_ms < (int64_t)timeout_ms;
    }
    pthread_mutex_unlock(&state_mutex);
    return alive;
}

/** @copydoc rov_command_state_set_gripper */
void rov_command_state_set_gripper(float input)
{
    pthread_mutex_lock(&state_mutex);
    state.gripper_input = input;
    pthread_mutex_unlock(&state_mutex);
}

/** @copydoc rov_command_state_set_auv_target */
void rov_command_state_set_auv_target(float depth, int has_depth,
                                      float heading, int has_heading)
{
    pthread_mutex_lock(&state_mutex);
    state.target_depth = depth;
    state.target_heading = heading;
    if (has_depth) state.depth_target_pending = 1;
    if (has_heading) state.heading_target_pending = 1;
    pthread_mutex_unlock(&state_mutex);
}

/** @copydoc rov_command_state_take_depth_target */
int rov_command_state_take_depth_target(float *target)
{
    int pending;
    if (target == NULL) return -1;
    pthread_mutex_lock(&state_mutex);
    *target = state.target_depth;
    pending = state.depth_target_pending;
    state.depth_target_pending = 0;
    pthread_mutex_unlock(&state_mutex);
    return pending;
}

/** @copydoc rov_command_state_take_heading_target */
int rov_command_state_take_heading_target(float *target)
{
    int pending;
    if (target == NULL) return -1;
    pthread_mutex_lock(&state_mutex);
    *target = state.target_heading;
    pending = state.heading_target_pending;
    state.heading_target_pending = 0;
    pthread_mutex_unlock(&state_mutex);
    return pending;
}

/** @copydoc rov_command_state_update_propellers */
void rov_command_state_update_propellers(
    const propeller_runtime_configs_t *propellers)
{
    if (propellers == NULL) return;
    pthread_mutex_lock(&state_mutex);
    state.propellers = *propellers;
    pthread_mutex_unlock(&state_mutex);
}
