/**
 * @desc: 创建 rpc服务端 线程
 */


#define LOG_TAG "rpc"

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <sys/wait.h>
#include <signal.h>
#include <pthread.h>
#include <elog.h>

#include "server.h"
#include "statusLight.h"

#include "../connections/crpcfunc.h"

struct jrpc_server my_server;

/**
 * @brief 向RPC服务器注册一个方法，并统一处理注册失败。
 */
static int register_rpc_method(jrpc_function function, const char *name)
{
    if (jrpc_register_procedure(&my_server, function, (char *)name, NULL) != 0)
    {
        log_e("Failed to register RPC method: %s", name);
        return -1;
    }

    return 0;
}

/**
  * @brief  信息回调函数
  */
static int info_init(void)
{
    return register_rpc_method(get_status_rpc_handler, "status")
        || register_rpc_method(get_propeller_power_rpc_handler, "propeller_power");
}

/**
  * @brief  控制回调函数
  */
static int control_init(void)
{
    return register_rpc_method(move_rpc_handler, "move")
        || register_rpc_method(set_gripper_rpc_handler, "gripper")
        || register_rpc_method(set_auv_target_rpc_handler, "auv_target");
}

/**
  * @brief  调试回调函数
  */
static int debug_init(void)
{
    return register_rpc_method(set_propeller_parameters_rpc_handler, "set_propeller_params")
        || register_rpc_method(get_pid_runtime_status_rpc_handler, "pid_runtime_status")
        || register_rpc_method(set_pid_rpc_handler, "set_pid")
        || register_rpc_method(load_parameters_rpc_handler, "load_params")
        || register_rpc_method(save_propeller_parameters_rpc_handler, "save_propeller_params")
        || register_rpc_method(save_pid_rpc_handler, "save_pid");
}

/**
  * @brief  rpc服务端线程
  */
static void *server_thread(void *arg)
{
    (void)arg;

    if (jrpc_server_init(&my_server, PORT) != 0)
    {
        log_e("Failed to start JsonRPC server on port %d", PORT);
        status_light_set(STATUS_LIGHT_FAULT);
        return NULL;
    }

    my_server.debug_level = 2;
    my_server.type = JRPC_TYPE_HTTP;

    if (info_init() != 0
        || control_init() != 0
        || debug_init() != 0)
    {
        log_e("JsonRPC method initialization failed");
        status_light_set(STATUS_LIGHT_FAULT);
        jrpc_server_stop(&my_server);
        jrpc_server_destroy(&my_server);
        return NULL;
    }

    log_i("JsonRPC server listening on port %d", my_server.port_number);
    jrpc_server_run(&my_server);
    jrpc_server_destroy(&my_server);
    return NULL;
}

/**
  * @brief  rpc服务端线程 初始化
  */
int server_thread_init(void)
{
    pthread_t server_tid;
    int result;

    log_i("JsonRPC init");
    result = pthread_create(&server_tid, NULL, server_thread, NULL);
    if (result != 0)
    {
        log_e("Failed to create JsonRPC thread: %s", strerror(result));
        return -1;
    }

    result = pthread_detach(server_tid);
    if (result != 0)
    {
        log_e("Failed to detach JsonRPC thread: %s", strerror(result));
        return -1;
    }

    return 0;
}
