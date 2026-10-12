/* 启用POSIX.1-2008接口声明，确保严格C11模式下可使用sigaction。 */
#define _POSIX_C_SOURCE 200809L

/**
 * @desc: 主程序
 */

#define LOG_TAG "main"

#include <errno.h>
#include <elog.h>
#include <signal.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "init.h"

static volatile sig_atomic_t running = 1;

/**
 * @brief  <CTRL>+C 程序中断回调
 */
static void signal_handler(int sig)
{
    (void)sig;
    running = 0;
}

int main(int argc, char **argv)
{
    /* 用来配置SIGINT和SIGTERM的处理方式。 */
    struct sigaction action;

    /* 当前程序不使用命令行参数，显式标记以避免编译器警告。 */
    (void)argc;
    (void)argv;

    /* 初始化日志、传感器、PWM、控制任务和RPC服务等全部模块。 */
    if (rov_init() < 0)
    {
        /* 关键模块初始化失败，返回非零值告诉Linux程序启动失败。 */
        return EXIT_FAILURE;
    }

    /* 先清空结构体，避免未设置字段含有随机值。 */
    memset(&action, 0, sizeof(action));

    /* 指定收到退出信号时调用signal_handler()。 */
    action.sa_handler = signal_handler;

    /* 信号处理函数执行期间不额外屏蔽其他信号。 */
    sigemptyset(&action.sa_mask);

    /*
     * SIGINT：用户在终端按Ctrl+C。
     * SIGTERM：systemd或kill命令请求程序正常退出。
     * 任意一个注册失败都不再继续运行，因为程序将无法可靠地安全停机。
     */
    if (sigaction(SIGINT, &action, NULL) < 0 || sigaction(SIGTERM, &action, NULL) < 0)
    {
        /* 记录信号注册失败的Linux错误原因。 */
        log_e("Could not register shutdown signal handler: %s", strerror(errno));

        /* 即使注册失败，也要先停止推进器再退出。 */
        rov_shutdown();

        /* 返回失败状态给Linux或systemd。 */
        return EXIT_FAILURE;
    }

    /* 上位机通过RPC服务交互；主线程仅等待系统退出信号。 */
    while (running)
    {
        /* 每秒检查一次退出标志，避免无意义的CPU忙循环。 */
        sleep(1);
    }

    /* 记录退出请求；此处已经离开信号回调，可以安全使用日志函数。 */
    log_i("Shutdown requested");

    /* 停止控制计算，将六路推进器输出回到零推力。 */
    rov_shutdown();

    /* 向Linux返回0，表示程序已完成正常安全退出。 */
    return EXIT_SUCCESS;
}
