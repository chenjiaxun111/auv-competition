/**
 * @file statusLight.c
 * @brief 使用RGB灯显示ROV运行状态。
 */

#define LOG_TAG "statusLight"

#include <elog.h>
#include <wiringPi.h>

#include "statusLight.h"

static void rgb_off(void)
{
    digitalWrite(LEDR_PIN, HIGH);
    digitalWrite(LEDG_PIN, HIGH);
    digitalWrite(LEDB_PIN, HIGH);
}

void status_light_set(status_light_state_t state)
{
    rgb_off();

    switch (state)
    {
        case STATUS_LIGHT_WAITING:
            digitalWrite(LEDB_PIN, LOW);
            break;
        case STATUS_LIGHT_NORMAL:
            digitalWrite(LEDG_PIN, LOW);
            break;
        case STATUS_LIGHT_FAULT:
            digitalWrite(LEDR_PIN, LOW);
            break;
        default:
            digitalWrite(LEDR_PIN, LOW);
            break;
    }
}

void status_light_init(void)
{
    pinMode(LEDR_PIN, OUTPUT);
    pinMode(LEDG_PIN, OUTPUT);
    pinMode(LEDB_PIN, OUTPUT);

    status_light_set(STATUS_LIGHT_WAITING);
    log_i("RGB status light initialized");
}
