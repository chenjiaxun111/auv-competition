#ifndef __MS5837_H_
#define __MS5837_H_

#include <stdint.h>

int ms5837_IIC_setup(const int pinBase);
int ms5837_IIC_data(int fd, float *temperature, float *pressure);
int ms5837_usart_setup(void);
int ms5837_usart_data(uint8_t data, float *temperature, float *depth);

#endif
