// stm8flash -c stlinkv2 -p stm8s103f3 -w main.ihx

#include <stm8s.h>

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

volatile u16 TIM4_tout;
volatile u16 sys_tout;
void main(void) {
    /* peripheral initialization */  
	#ifdef FAST_I2C_MODE
        CLK->CKDIVR = 0x00;             // sys clock / 1
    #else
        CLK->CKDIVR = 0x01;             // sys clock / 2
    #endif

    TIM4_Init();   // initialize timer 4 mandatory for timout and tick measurement    
    uart1_config();

    I2C_DeInit();
    I2C_Setting();   // Initialize I2C for communication

    // Enable Interrupt
    __asm__("rim"); // interrupt enable
    set_sys_tout_ms(1000);
    while(1){


        if(sys_tout()==0) { // system timer loop 
            
            uart_write("LP!! \r\n");
            
            set_tout_ms(100);
            if(tout()){
                set_tout_ms(100);
                uint8_t i2c_read_data=I2C_ReadByte(0x33);
                UART1_SendRegisterValue("I2C READ ",i2c_read_data );
            } 


            set_sys_tout_ms(1000);
           
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
    
    if(sys_tout){
        --sys_tout;
    }

  }