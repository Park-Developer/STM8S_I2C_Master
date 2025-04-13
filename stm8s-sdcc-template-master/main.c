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
    set_sys_tout_ms(2000);
    
    uint16_t i2c_2byte;

    // INA219 Config
    INA219 ina219;
    ina219.device_no=1;
    
    ina219.i2c_addr=0x40;
    ina219.VBUS_Max=32; // (V)
    ina219.ADC_RES=4096; // 12 bit resolution
    ina219.R_Shunt=0.1; // (Ohm)
    ina219.Max_possible_Curr=3.2; // (A)
    ina219.Calibration_Value=524; // calculated with 0.04096
    ina219.Current_LSB_mA=0.781;
    
    
    //set_INA219(ina219); // INA219 Setting
    uint8_t test[2]={0x2c,0x3d};

    while(1){


        if(sys_tout()==0) { // system timer loop 
            
            uart_write("LP1!! \r\n");
            
            set_tout_ms(500);
            if(tout()){
                set_tout_ms(500);
                
                //I2C_ReadRegister(0x08, INA219_REG_CALIBRATION, 2, test);
                //UART1_SendRegisterValue("CAL1", test[1]);
                //UART1_SendRegisterValue("CAL1", test[0]);

                //I2C_ReadRegister(ina219.i2c_addr, INA219_REG_CURRENT, 2, test);
                //UART1_SendRegisterValue("CUR1", test[1]);
                //UART1_SendRegisterValue("CUR1", test[0]);
                //uint8_t i2c_read_data=I2C_ReadByte(0x08); => ok

                //uint8_t i2c_read_data=I2C_ReadByte(0x08);// => ok
                //UART1_SendRegisterValue("read",i2c_read_data);

                I2C_WriteRegister(0x08, 0x12, 2, test);

                i2c_2byte=I2C_Read_2Bytes(0x08);
                UART1_SendRegisterValue("CUR1", (uint8_t)(i2c_2byte>>8));
                UART1_SendRegisterValue("CUR2", (uint8_t)i2c_2byte);
       
            } 


            set_sys_tout_ms(2000);
           
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