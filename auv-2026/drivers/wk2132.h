/**
 * @file wk2132.h
 * @brief WK2132双通道UART扩展芯片的预留驱动接口。
 *
 * @note 当前项目未调用本驱动，仅为以后扩展串口设备保留。
 * @warning 接入前需核对I2C总线、晶振频率和子串口寄存器寻址，
 * 并完成实物验证。
 */

#ifndef WK2132_H
#define WK2132_H

#include <stdint.h>

uint8_t wk2132SendRaw(int fd, const uint8_t *data, uint8_t length);
uint8_t wk2132SendStr(int fd, const char *data);
int wk2132RecvByte(int fd);
uint8_t wk2132DataAvail(int fd);
uint8_t wk2132SetUartBaud(int fd, uint32_t baud);
int wk2132Setup(void);

#endif
