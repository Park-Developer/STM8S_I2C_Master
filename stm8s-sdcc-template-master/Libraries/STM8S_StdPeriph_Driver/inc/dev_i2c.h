#ifndef __STM8S_DEV_I2C_H
#define __STM8S_DEV_I2C_H

#include "stm8s.h"
void TIM4_Init (void);

extern volatile u16 TIM4_tout;
extern volatile u16 sys_tout;
/* flag clearing sequence - uncoment next for peripheral clock under 2MHz */
#define dead_time() { /* _asm("nop"); _asm("nop"); */ }
#define delay(a)          { TIM4_tout= a; while(TIM4_tout); }

#define tout()            (TIM4_tout)
#define set_tout_ms(a)    { TIM4_tout= a; }


#define sys_tout() (sys_tout)
#define set_sys_tout_ms(a) {sys_tout=a;}

#define SLAVE_ADDRESS  0x33

void I2C_Setting(void);

void I2C_ReadRegister(u8 u8_regAddr, u8 u8_NumByteToRead, u8 *u8_DataBuffer);

void I2C_WriteRegister(u8 u8_regAddr, u8 u8_NumByteToWrite, u8 *u8_DataBuffer);

uint8_t I2C_ReadByte(uint8_t slaveAddr);

void ErrProc(void);
#endif