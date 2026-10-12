#ifndef STATUS_LIGHT_H
#define STATUS_LIGHT_H

#include "../user/config.h"

/* RGB灯使用wiringPi引脚号，共阳极：低电平点亮，高电平熄灭。 */
#define LEDR_PIN 13
#define LEDG_PIN 14
#define LEDB_PIN 10

typedef enum
{
    STATUS_LIGHT_WAITING, /* 蓝色：等待上位机连接 */
    STATUS_LIGHT_NORMAL,  /* 绿色：正常运行 */
    STATUS_LIGHT_FAULT    /* 红色：发生故障 */
} status_light_state_t;

void status_light_init(void);
void status_light_set(status_light_state_t state);

#endif
