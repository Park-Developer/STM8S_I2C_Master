/********************************************************************
 * Copyright 2017 Ahmet Onat
 * This program is distributed under the terms of the 
 * GNU General Public License
 *
 * This file is part of SDCC_StmPeriphLib
 *
 * SDCC_StmPeriphLib is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * SDCC_StmPeriphLib is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * You should have received a copy of the GNU General Public License
 * along with SDCC_StmPeriphLib.  If not, see <http://www.gnu.org/licenses/>.
 *
 *********************************************************************/

/*
stm8flash -c stlinkv2 -p stm8s103f3 -w main.ihx
*/


#include "stm8s.h"
#include "stm8s_it.h"
#include "stm8s_gpio.h"
#include "stm8s_i2c.h"
#include "stm8s_clk.h"
#include "stm8s_exti.h"
#include "stm8s_uart1.h"
#include "stm8s_adc1.h"
#include "stm8s_tim4.h"

#include <string.h> 
#include <stdio.h>

#include "dev_uart.h"
#include "dev_timer.h"
#include "dev_i2c.h"

// TIMER SETTING
#define tout()            (TIM4_tout)
#define set_tout_ms(a)    { TIM4_tout= a; }
u16 TIM4_tout;

/* GPIO Setting */
#define LED_PORT  GPIOB
#define LED_PIN   GPIO_PIN_5



void main(void)
{
  // TIMER 
  TIM4_Init();
  CLK->CKDIVR = 0x00; 
  //GPIO_Init(LED_PORT, LED_PIN, GPIO_MODE_OUT_PP_LOW_FAST);
  GPIO_DeInit(GPIOB);
  GPIO_DeInit(GPIOD);

  GPIO_Init(GPIOD, GPIO_PIN_4, GPIO_MODE_OUT_PP_LOW_FAST);

  GPIO_WriteHigh(GPIOD, GPIO_PIN_4);


  // uart gpio
  uart1_config();

  //Initialize I2C in slave mode
  I2C_DeInit();
  I2C_Init_Master();
  GPIO_Init(GPIOB, GPIO_PIN_4, GPIO_MODE_IN_FL_NO_IT);
  GPIO_Init(GPIOB, GPIO_PIN_5, GPIO_MODE_IN_FL_NO_IT);
  

    __asm__("rim"); // interrupt enable

    set_tout_ms(1000);
  
    uint8_t i2c_read_data;
    
    while (1){
    if(tout()==0){
      uart_write("LP!! \r\n");
      set_tout_ms(1000);

      i2c_read_data=I2C_ReadByte(0x40);
      UART1_SendByte(i2c_read_data);
    }
  
  }
}



void UART1_RX_IRQHandler(void) __interrupt(18) {
  uint8_t received_data;
  if (UART1_GetFlagStatus(UART1_FLAG_RXNE)) {
      received_data = UART1_ReceiveData8();
      UART1_ClearFlag(UART1_FLAG_RXNE);

      UART1_SendByte(received_data);   
  }
}


void TIM4_UPD_OVF_IRQHandler(void) __interrupt(23){
  u8 dly= 10;
  
  TIM4->SR1= 0;
  
  if(TIM4_tout){
    --TIM4_tout;
  }
  
}