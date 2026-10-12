/**
 * @file wk2132.c
 * @brief WK2132子串口1的基础I2C驱动。
 * @note 当前项目未调用本文件中的函数，仅保留为未来硬件扩展接口。
 */

#define LOG_TAG "wk2132"

#include <elog.h>
#include <stddef.h>
#include <stdint.h>
#include <unistd.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>

#include "wk2132.h"

#define WK2132_CLK_HZ       22118400UL
#define WK2132_I2C_DEVICE   "/dev/i2c-1"
#define WK2132_I2C_ADDRESS  0x70
#define WK2132_TX_TIMEOUT_MS 1000U
#define WK2132_POLL_MS        10U

#define WK2132_REG_GENA  0x00
#define WK2132_REG_GRST  0x01
#define WK2132_REG_SPAGE 0x03
#define WK2132_REG_SCR   0x04
#define WK2132_REG_FCR   0x06
#define WK2132_REG_TFCNT 0x09
#define WK2132_REG_RFCNT 0x0A
#define WK2132_REG_FDAT  0x0D

/* SPAGE=1时，以下地址分别表示波特率整数高/低字节和小数部分。 */
#define WK2132_REG_BAUD1 0x04
#define WK2132_REG_BAUD0 0x05
#define WK2132_REG_PRES  0x06

#define WK2132_UART1_ENABLE 0x01
#define WK2132_UART2_ENABLE 0x02
#define WK2132_UART_ENABLE_ALL (WK2132_UART1_ENABLE | WK2132_UART2_ENABLE)

static int wk2132_write_register(int fd, int reg, int value)
{
    if (fd < 0 || wiringPiI2CWriteReg8(fd, reg, value) < 0)
    {
        log_e("WK2132 register write failed: reg=0x%02X", reg);
        return -1;
    }
    return 0;
}

static int wk2132_read_register(int fd, int reg)
{
    int value;

    if (fd < 0)
        return -1;

    value = wiringPiI2CReadReg8(fd, reg);
    if (value < 0)
        log_e("WK2132 register read failed: reg=0x%02X", reg);
    return value;
}

static int wk2132_wait_tx_complete(int fd)
{
    unsigned int waited_ms = 0;
    int count;

    while (waited_ms < WK2132_TX_TIMEOUT_MS)
    {
        count = wk2132_read_register(fd, WK2132_REG_TFCNT);
        if (count < 0)
            return -1;
        if (count == 0)
            return 0;

        delay(WK2132_POLL_MS);
        waited_ms += WK2132_POLL_MS;
    }

    log_e("WK2132 transmit timeout");
    return -1;
}

uint8_t wk2132SendStr(int fd, const char *data)
{
    if (data == NULL)
        return 0;

    while (*data != '\0')
    {
        if (wk2132_write_register(fd, WK2132_REG_FDAT, (uint8_t)*data) < 0)
            return 0;
        data++;
    }

    return wk2132_wait_tx_complete(fd) == 0;
}

uint8_t wk2132SendRaw(int fd, const uint8_t *data, uint8_t length)
{
    uint8_t i;

    if (data == NULL && length != 0)
        return 0;

    for (i = 0; i < length; i++)
    {
        if (wk2132_write_register(fd, WK2132_REG_FDAT, data[i]) < 0)
            return 0;
    }

    return wk2132_wait_tx_complete(fd) == 0;
}

int wk2132RecvByte(int fd)
{
    return wk2132_read_register(fd, WK2132_REG_FDAT);
}

uint8_t wk2132DataAvail(int fd)
{
    int count = wk2132_read_register(fd, WK2132_REG_RFCNT);
    return count > 0;
}

uint8_t wk2132SetUartBaud(int fd, uint32_t baud)
{
    uint64_t scaled_divisor;
    uint32_t integer_part;
    uint32_t fraction;
    int failed = 0;

    if (fd < 0 || baud == 0)
        return 0;

    scaled_divisor = ((uint64_t)WK2132_CLK_HZ * 100U) / (16U * baud);
    if (scaled_divisor < 100U)
        return 0;

    integer_part = (uint32_t)(scaled_divisor / 100U) - 1U;
    fraction = (uint32_t)(((scaled_divisor % 100U) * 16U + 50U) / 100U);
    if (fraction >= 16U)
    {
        integer_part++;
        fraction = 0;
    }
    if (integer_part > UINT16_MAX)
        return 0;

    if (wk2132_write_register(fd, WK2132_REG_SPAGE, 1) < 0)
        return 0;

    failed |= wk2132_write_register(fd, WK2132_REG_BAUD1, integer_part >> 8) < 0;
    failed |= wk2132_write_register(fd, WK2132_REG_BAUD0, integer_part & 0xFF) < 0;
    failed |= wk2132_write_register(fd, WK2132_REG_PRES, fraction) < 0;
    failed |= wk2132_write_register(fd, WK2132_REG_SPAGE, 0) < 0;
    return failed == 0;
}

static int wk2132_register_init(int fd)
{
    if (wk2132_write_register(fd, WK2132_REG_GENA, WK2132_UART_ENABLE_ALL) < 0
        || wk2132_write_register(fd, WK2132_REG_GRST, WK2132_UART_ENABLE_ALL) < 0
        || wk2132_write_register(fd, WK2132_REG_SPAGE, 0) < 0
        || wk2132_write_register(fd, WK2132_REG_SCR, 0x03) < 0
        || wk2132_write_register(fd, WK2132_REG_FCR, 0x03) < 0
        || wk2132_write_register(fd, WK2132_REG_FCR, 0x0C) < 0)
        return -1;

    return 0;
}

int wk2132Setup(void)
{
    int fd = wiringPiI2CSetupInterface(WK2132_I2C_DEVICE, WK2132_I2C_ADDRESS);

    if (fd < 0)
    {
        log_e("Unable to open WK2132 on %s", WK2132_I2C_DEVICE);
        return fd;
    }

    if (wk2132_register_init(fd) < 0)
    {
        log_e("Unable to initialize WK2132 registers");
        close(fd);
        return -1;
    }

    log_i("WK2132 initialized");
    return fd;
}
