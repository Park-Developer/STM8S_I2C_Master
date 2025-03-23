#ifndef __DEV_UART_H
#define __DEV_UART_H


#include "stm8s.h"
void uart1_config(void);

int uart_write(const char *str);
void UART1_SendByte(uint8_t byte);
void UART1_SendString(const char *str);
void UART1_SendRegisterValue(const char *regName, uint8_t regValue);
void UART1_Send16(uint16_t value) ;

#endif