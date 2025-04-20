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

volatile uint8_t received_data;
volatile uint8_t ina219_recv[2];
volatile uint16_t curr_val;

// INA219 Config
INA219 ina219;




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
    
    
    // Set INA219
    ina219.device_no=1;    
    ina219.i2c_addr=0x40;
    ina219.VBUS_Max=32; // (V)
    ina219.ADC_RES=4096; // 12 bit resolution
    ina219.R_Shunt=0.1; // (Ohm)
    ina219.Max_possible_Curr=3.2; // (A)
    ina219.Calibration_Value=524; // calculated with 0.04096
    ina219.Current_LSB_mA=0.781;
    
    set_tout_ms(500);
    set_INA219(ina219);

    
    while(1){

        if(sys_tout()==0) { // system timer loop 
            
            uart_write("LP\r\n");
            
            set_tout_ms(500);
            if(tout()){
                set_tout_ms(500);
                
               

               


                
               

            
            } 


            set_sys_tout_ms(2000);
           
        }
    }

}

void UART1_RX_IRQHandler(void) __interrupt(18) {
    
    if (UART1_GetFlagStatus(UART1_FLAG_RXNE)) {
        // Receive Data
        received_data = UART1_ReceiveData8();
        UART1_ClearFlag(UART1_FLAG_RXNE);
  

        if(received_data==0xC1){ // read config register
             
            I2C_ReadRegister(INA219_ADDR, INA219_REG_CONFIG, 2, ina219_recv);

            UART1_SendRegisterValue("INA219_CONFIG MS Byte", ina219_recv[0]);
            UART1_SendRegisterValue("INA219_CONFIG LS Byte", ina219_recv[1]);

        }else if(received_data==0xC2){ // read calibration register
            
            I2C_ReadRegister(INA219_ADDR, INA219_REG_CALIBRATION, 2, ina219_recv);

            UART1_SendRegisterValue("INA219_CALI MS Byte", ina219_recv[0]);
            UART1_SendRegisterValue("INA219_CALI LS Byte", ina219_recv[1]);

        }else if(received_data==0xC3){ // read current register

            curr_val=getCurrent_mA(ina219);
        
            UART1_SendRegisterValue("INA219_CURR MS Byte", (uint8_t)(curr_val>>8));
            UART1_SendRegisterValue("INA219_CURR LS Byte", (uint8_t)(curr_val));
        
        }else if(received_data==0xE1){
            // Arduino IF
            uint8_t test[2]={0x2c,0x3d};
            I2C_WriteRegister(0x08, 0x12, 2, test);
            uint16_t arduino_i2c_2byte;

            arduino_i2c_2byte=I2C_Read_2Bytes(0x08);
            
            UART1_SendRegisterValue("ARDUINO 1", (uint8_t)(arduino_i2c_2byte>>8));
            UART1_SendRegisterValue("ARDUINO 2", (uint8_t)arduino_i2c_2byte);
        }


        // Transmit Data
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