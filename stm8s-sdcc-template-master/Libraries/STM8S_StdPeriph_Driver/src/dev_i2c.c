#include "dev_i2c.h"

extern int uart_write(const char *str);
extern void UART1_SendByte(uint8_t byte);

/******************************************************************************
* Function name : I2C_Init
* Description 	: Initialize I2C peripheral
* Input param 	: None
* Return 		    : None
* See also 		  : None
*******************************************************************************/
void I2C_Setting(void) {
  GPIO_Init(GPIOB, GPIO_PIN_4, GPIO_MODE_IN_FL_NO_IT);
  GPIO_Init(GPIOB, GPIO_PIN_5, GPIO_MODE_IN_FL_NO_IT);
  
  CLK_PeripheralClockConfig(CLK_PERIPHERAL_I2C, ENABLE);

#ifdef FAST_I2C_MODE
  I2C->FREQR = 16;               // input clock to I2C - 16MHz 
  I2C->CCRL = 15;                // 900/62.5= 15, (SCLhi must be at least 600+300=900ns!)
  I2C->CCRH = 0x80;              // fast mode, duty 2/1 (bus speed 62.5*3*15~356kHz)
  I2C->TRISER = 5;               // 300/62.5 + 1= 5  (maximum 300ns)
#else
  I2C->FREQR = 8;                // input clock to I2C - 8MHz
  I2C->CCRL = 40;                // CCR= 40 - (SCLhi must be at least 4000+1000=5000ns!)
  I2C->CCRH = 0;                 // standard mode, duty 1/1 bus speed 100kHz
  I2C->TRISER = 9;               // 1000ns/(125ns) + 1  (maximum 1000ns)
#endif
  I2C->OARL = 0xA0;              // own address A0;
  I2C->OARH |= 0x40;
  I2C->ITR = 1;                  // enable error interrupts
  I2C->CR2 |= 0x04;              // ACK=1, Ack enable
  I2C->CR1 |= 0x01;              // PE=1
}


/******************************************************************************
* Function name : I2C_ReadRegister
* Description 	: Read defined number bytes from slave memory starting with defined offset
* Input param 	: offset in slave memory, number of bytes to read, starting address to store received data
* Return 		    : None
* See also 		  : None
*******************************************************************************/
void I2C_ReadRegister(u8 slave_address, u8 u8_regAddr, u8 u8_NumByteToRead, u8 *u8_DataBuffer)
{
  /*--------------- BUSY? -> STOP request ---------------------*/
	while(I2C->SR3 & I2C_SR3_BUSY  &&  tout())	  				// Wait while the bus is busy
  {
		I2C->CR2 |= I2C_CR2_STOP;                   				// Generate stop here (STOP=1)
    while(I2C->CR2 & I2C_CR2_STOP  &&  tout()); 				// Wait until stop is performed
	}
  I2C->CR2 |= I2C_CR2_ACK;                      				// ACK=1, Ack enable
  /*--------------- Start communication -----------------------*/  
  I2C->CR2 |= I2C_CR2_START;                    				// START=1, generate start
  while((I2C->SR1 & I2C_SR1_SB)==0  &&  tout());				// Wait for start bit detection (SB)
  /*------------------ Address send ---------------------------*/      
  if(tout())
  {
    #ifdef TEN_BITS_ADDRESS
      I2C->DR = (u8)(((slave_address >> 7) & 6) | 0xF0);// Send header of 10-bit device address (R/W = 0)
      while(!(I2C->SR1 & I2C_SR1_ADD10) &&  tout());		// Wait for header ack (ADD10)
      if(tout())
      {
        I2C->DR = (u8)(slave_address);                	// Send lower 8-bit device address & Write  
      }
    #else
      I2C->DR = (u8)(slave_address << 1);   						// Send 7-bit device address & Write (R/W = 0)
    #endif // TEN_BITS_ADDRESS
  }
  while(!(I2C->SR1 & I2C_SR1_ADDR) &&  tout()); 				// test EV6 - wait for address ack (ADDR)
  dead_time();                                  				// ADDR clearing sequence
  I2C->SR3;

  /*--------------- Register/Command send ----------------------*/
  while(!(I2C->SR1 & I2C_SR1_TXE) &&  tout());  				// Wait for TxE
  if(tout())
  {  
    I2C->DR = u8_regAddr;                         			// Send register address
  }        
  
  // Wait for TxE & BTF
  while((I2C->SR1 & (I2C_SR1_TXE | I2C_SR1_BTF)) != (I2C_SR1_TXE | I2C_SR1_BTF)  &&  tout()); 
  dead_time();                                  				// clearing sequence
  /*-------------- Stop/Restart communication -------------------*/  
  #ifndef TEN_BITS_ADDRESS
    #ifdef NO_RESTART																		// if 7bit address and NO_RESTART setted
      I2C->CR2 |= I2C_CR2_STOP;                     		// STOP=1, generate stop
      while(I2C->CR2 & I2C_CR2_STOP  &&  tout());   		// wait until stop is performed
    #endif // NO_RESTART
  #endif // TEN_BITS_ADDRESS
  /*--------------- Restart communication ---------------------*/  
  I2C->CR2 |= I2C_CR2_START;                     				// START=1, generate re-start
  while((I2C->SR1 & I2C_SR1_SB)==0  &&  tout()); 				// Wait for start bit detection (SB)
  /*------------------ Address send ---------------------------*/      
  if(tout())
  {
    #ifdef TEN_BITS_ADDRESS
      I2C->DR = (u8)(((slave_address >> 7) & 6) | 0xF1);// send header of 10-bit device address (R/W = 1)
      #ifdef NO_RESTART
        while(!(I2C->SR1 & I2C_SR1_ADD10) &&  tout());	// Wait for header ack (ADD10)
        if(tout())
        {
          I2C->DR = (u8)(slave_address);                // Send lower 8-bit device address & Write  
        }
      #endif // NO_RESTART
    #else
      I2C->DR = (u8)(slave_address << 1) | 1;         	// Send 7-bit device address & Write (R/W = 1)
    #endif  // TEN_BITS_ADDRESS
  }
  while(!(I2C->SR1 & I2C_SR1_ADDR)  &&  tout());  			// Wait for address ack (ADDR)
  /*------------------- Data Receive --------------------------*/
  if (u8_NumByteToRead > 2)                 						// *** more than 2 bytes are received? ***
  {
    I2C->SR3;                                     			// ADDR clearing sequence    
    while(u8_NumByteToRead > 3  &&  tout())       			// not last three bytes?
    {
      while(!(I2C->SR1 & I2C_SR1_BTF)  &&  tout()); 				// Wait for BTF
			*u8_DataBuffer++ = I2C->DR;                   				// Reading next data byte
      --u8_NumByteToRead;																		// Decrease Numbyte to reade by 1
    }
																												//last three bytes should be read
    while(!(I2C->SR1 & I2C_SR1_BTF)  &&  tout()); 			// Wait for BTF
    I2C->CR2 &=~I2C_CR2_ACK;                      			// Clear ACK
    __asm__("sim");                        			// Errata workaround (Disable interrupt)
    *u8_DataBuffer++ = I2C->DR;                     		// Read 1st byte
    I2C->CR2 |= I2C_CR2_STOP;                       		// Generate stop here (STOP=1)
    *u8_DataBuffer++ = I2C->DR;                     		// Read 2nd byte
    __asm__("rim");																	// Errata workaround (Enable interrupt)
    while(!(I2C->SR1 & I2C_SR1_RXNE)  &&  tout());			// Wait for RXNE
    *u8_DataBuffer++ = I2C->DR;                   			// Read 3rd Data byte
  }
  else
  {
   if(u8_NumByteToRead == 2)                						// *** just two bytes are received? ***
    {
      I2C->CR2 |= I2C_CR2_POS;                      		// Set POS bit (NACK at next received byte)
      __asm__("sim");                           		// Errata workaround (Disable interrupt)
      I2C->SR3;                                       	// Clear ADDR Flag
      I2C->CR2 &=~I2C_CR2_ACK;                        	// Clear ACK 
      __asm__("rim");																	// Errata workaround (Enable interrupt)
      while(!(I2C->SR1 & I2C_SR1_BTF)  &&  tout()); 		// Wait for BTF
      __asm__("sim");                          		// Errata workaround (Disable interrupt)
      I2C->CR2 |= I2C_CR2_STOP;                       	// Generate stop here (STOP=1)
      *u8_DataBuffer++ = I2C->DR;                     	// Read 1st Data byte
      __asm__("rim");																// Errata workaround (Enable interrupt)
			*u8_DataBuffer = I2C->DR;													// Read 2nd Data byte
    }
    else                                      					// *** only one byte is received ***
    {
      I2C->CR2 &=~I2C_CR2_ACK;;                     		// Clear ACK 
      __asm__("sim");                            		// Errata workaround (Disable interrupt)
      I2C->SR3;                                       	// Clear ADDR Flag   
      I2C->CR2 |= I2C_CR2_STOP;                       	// generate stop here (STOP=1)
      __asm__("rim");																		// Errata workaround (Enable interrupt)
      while(!(I2C->SR1 & I2C_SR1_RXNE)  &&  tout()); 		// test EV7, wait for RxNE
      *u8_DataBuffer = I2C->DR;                     		// Read Data byte
    }
  }
  /*--------------- All Data Received -----------------------*/
  while((I2C->CR2 & I2C_CR2_STOP)  &&  tout());     		// Wait until stop is performed (STOPF = 1)
  I2C->CR2 &=~I2C_CR2_POS;                          		// return POS to default state (POS=0)
}

/******************************************************************************
* Function name : I2C_WriteRegister
* Description 	: write defined number bytes to slave memory starting with defined offset
* Input param 	: offset in slave memory, number of bytes to write, starting address to send
* Return 		    : None.
* See also 		  : None.
*******************************************************************************/
void I2C_WriteRegister(u8 slave_address, u8 u8_regAddr, u8 u8_NumByteToWrite, u8 *u8_DataBuffer)
{
  while((I2C->SR3 & 2) && tout())       									// Wait while the bus is busy
  {
    I2C->CR2 |= 2;                        								// STOP=1, generate stop
    while((I2C->CR2 & 2) && tout());      								// wait until stop is performed
  }
  
  I2C->CR2 |= 1;                        									// START=1, generate start
  while(((I2C->SR1 & 1)==0) && tout()); 									// Wait for start bit detection (SB)
  dead_time();                          									// SB clearing sequence
  if(tout())
  {
    #ifdef TEN_BITS_ADDRESS															  // TEN_BIT_ADDRESS decalred in I2c_master_poll.h
      I2C->DR = (u8)(((slave_address >> 7) & 6) | 0xF0);  // Send header of 10-bit device address (R/W = 0)
      while(!(I2C->SR1 & 8) &&  tout());    							// Wait for header ack (ADD10)
      if(tout())
      {
        I2C->DR = (u8)(slave_address);        						// Send lower 8-bit device address & Write 
      }
    #else
      I2C->DR = (u8)(slave_address << 1);   							// Send 7-bit device address & Write (R/W = 0)
    #endif
  }
  while(!(I2C->SR1 & 2) && tout());     									// Wait for address ack (ADDR)
  dead_time();                          									// ADDR clearing sequence
  I2C->SR3;
  while(!(I2C->SR1 & 0x80) && tout());  									// Wait for TxE
  if(tout())
  {
    I2C->DR = u8_regAddr;                 								// send Offset command
  }
  if(u8_NumByteToWrite)
  {
    while(u8_NumByteToWrite--)          									
    {																											// write data loop start
      while(!(I2C->SR1 & 0x80) && tout());  								// test EV8 - wait for TxE
      I2C->DR = *u8_DataBuffer++;           								// send next data byte
    }																											// write data loop end
  }
  while(((I2C->SR1 & 0x84) != 0x84) && tout()); 					// Wait for TxE & BTF
  dead_time();                          									// clearing sequence
  
  I2C->CR2 |= 2;                        									// generate stop here (STOP=1)
  while((I2C->CR2 & 2) && tout());      									// wait until stop is performed  
}


uint8_t I2C_ReadByte(uint8_t slaveAddr) {
  uint8_t data;

  while (I2C_GetFlagStatus(I2C_FLAG_BUSBUSY) && --tout());
  if(tout()==0){
    return 0xf1;
  }

  I2C_GenerateSTART(ENABLE);

  while (!I2C_CheckEvent(I2C_EVENT_MASTER_MODE_SELECT) && --tout());
  if(tout()==0){
    return 0xf2;
  }
  
 
  I2C_Send7bitAddress((slaveAddr << 1), I2C_DIRECTION_RX);
  while (!I2C_CheckEvent(I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED) && --tout());
  if(tout()==0){
    return 0xf3;
  }
  I2C_AcknowledgeConfig(I2C_ACK_NONE);
  I2C_GenerateSTOP(ENABLE);

  while (!I2C_CheckEvent(I2C_EVENT_MASTER_BYTE_RECEIVED) && --tout());
  if(tout()==0){
    return 0xf4;
  }
  data = I2C_ReceiveData();
  
  return data;
}

uint16_t I2C_Read_2Bytes(uint8_t slave_address)
{
    uint8_t low_byte, high_byte;
    uint16_t result;

    // Wait if bus is busy
    while (I2C->SR3 & I2C_SR3_BUSY);
    uart_write("tp1");
    I2C->CR2 |= I2C_CR2_ACK;              // Enable ACK
    I2C->CR2 |= I2C_CR2_START;            // Generate START
    while (!(I2C->SR1 & I2C_SR1_SB));     // Wait for SB

    // Send slave address with read bit
    I2C->DR = (slave_address << 1) | 1;
    while (!(I2C->SR1 & I2C_SR1_ADDR));   // Wait for address acknowledged
    uart_write("tp22");
    I2C->CR2 |= I2C_CR2_POS;              // Prepare NACK for next byte
    I2C->SR3;                             // Clear ADDR by reading SR3
    I2C->CR2 &= ~I2C_CR2_ACK;             // Disable ACK

    while (!(I2C->SR1 & I2C_SR1_BTF));    // Wait for 2 bytes received
    uart_write("tp3");
    I2C->CR2 |= I2C_CR2_STOP;             // Generate STOP
    low_byte = I2C->DR;                   // Read first byte (LSB)
    high_byte = I2C->DR;                  // Read second byte (MSB)

    result = ((uint16_t)high_byte << 8) | low_byte;

    return result;
}

/******************************************************************************
* Function name : I2C_RandomRead
* Description 	: Read defined number bytes from slave memory starting with curret offset
* Input param 	: number of bytes to read, starting address to store received data
* Return 		    : None
* See also 		  : None
*******************************************************************************/
void I2C_RandomRead(u8 u8_NumByteToRead, u8 *u8_DataBuffer, u8 slave_address) 
{
  /*--------------- BUSY? -> STOP request ---------------------*/
	while(I2C->SR3 & I2C_SR3_BUSY  &&  tout())	  				// Wait while the bus is busy
  {
		I2C->CR2 |= I2C_CR2_STOP;                   				// STOP=1, generate stop
    while(I2C->CR2 & I2C_CR2_STOP  &&  tout()); 				// wait until stop is performed
	}
  I2C->CR2 |= I2C_CR2_ACK;                      				// ACK=1, Ack enable
  /*--------------- Start communication -----------------------*/  
  I2C->CR2 |= I2C_CR2_START;                    				// START=1, generate start
  while((I2C->SR1 & I2C_SR1_SB)==0  &&  tout());				// wait for start bit detection (SB)
  /*------------------ Address send ---------------------------*/      
  #ifdef TEN_BITS_ADDRESS
    I2C->DR = (u8)(((slave_address >> 7) & 6) | 0xF0);	// Send header of 10-bit device address (R/W = 0)
    while(!(I2C->SR1 & I2C_SR1_ADD10) &&  tout());			// Wait for header ack (ADD10)
    I2C->DR = (u8)(slave_address);                			// Send lower 8-bit device address
		while(!(I2C->SR1 & I2C_SR1_ADDR)  &&  tout());			// Wait for address ack (ADDR)
		I2C->CR2 |= I2C_CR2_START;                    			// START=1, generate start
		while((I2C->SR1 & I2C_SR1_SB)==0  &&  tout());			// Wait for start bit detection (SB)
		I2C->DR = (u8)(((slave_address >> 7) & 6) | 0xF1);	// Send header of 10-bit device address (R/W = 1)
	#else
    I2C->DR = (u8)(slave_address << 1) | 1;       			// Send 7-bit device address & Write (R/W = 1)
  #endif  // TEN_BITS_ADDRESS
  while(!(I2C->SR1 & I2C_SR1_ADDR)  &&  tout());				// Wait for address ack (ADDR)
  /*------------------- Data Receive --------------------------*/
  if (u8_NumByteToRead > 2)                 						// *** more than 2 bytes are received? ***
  {
    I2C->SR3;                                     			// ADDR clearing sequence    
    while(u8_NumByteToRead > 3  &&  tout())       			// not last three bytes?
    {
      while(!(I2C->SR1 & I2C_SR1_BTF)  &&  tout()); 				// Wait for BTF
			*u8_DataBuffer++ = I2C->DR;                   				// Reading next data byte
      --u8_NumByteToRead;																		// Decrease Numbyte to reade by 1
    }
																												//last three bytes should be read
    while(!(I2C->SR1 & I2C_SR1_BTF)  &&  tout()); 			// Wait for BTF
    I2C->CR2 &=~I2C_CR2_ACK;                      			// Clear ACK
    __asm__("sim");                         			// Errata workaround (Disable interrupt)
    *u8_DataBuffer++ = I2C->DR;                     		// Read 1st byte
    I2C->CR2 |= I2C_CR2_STOP;                       		// Generate stop here (STOP=1)
    *u8_DataBuffer++ = I2C->DR;                     		// Read 2nd byte
    __asm__("rim"); 																// Errata workaround (Enable interrupt)
    while(!(I2C->SR1 & I2C_SR1_RXNE)  &&  tout());			// Wait for RXNE
    *u8_DataBuffer++ = I2C->DR;                   			// Read 3rd Data byte
  }
  else
  {
    if(u8_NumByteToRead == 2)                						// *** just two bytes are received? ***
    {
      I2C->CR2 |= I2C_CR2_POS;                      		// Set POS bit (NACK at next received byte)
      __asm__("sim");                            		// Errata workaround (Disable interrupt)
      I2C->SR3;                                       	// Clear ADDR Flag
      I2C->CR2 &=~I2C_CR2_ACK;                        	// Clear ACK 
      __asm__("rim"); 															// Errata workaround (Enable interrupt)
      while(!(I2C->SR1 & I2C_SR1_BTF)  &&  tout()); 		// Wait for BTF
      __asm__("sim");                            		// Errata workaround (Disable interrupt)
      I2C->CR2 |= I2C_CR2_STOP;                       	// Generate stop here (STOP=1)
      *u8_DataBuffer++ = I2C->DR;                     	// Read 1st Data byte
      UART1_SendByte(I2C->DR); //DEBUG
      __asm__("rim"); 															// Errata workaround (Enable interrupt)
			*u8_DataBuffer = I2C->DR;													// Read 2nd Data byte
       UART1_SendByte(I2C->DR); //DEBUG
    }
    else                                      					// *** only one byte is received ***
    {
      I2C->CR2 &=~I2C_CR2_ACK;;                     		// Clear ACK 
      __asm__("sim");                          		// Errata workaround (Disable interrupt)
      I2C->SR3;                                       	// Clear ADDR Flag   
      I2C->CR2 |= I2C_CR2_STOP;                       	// generate stop here (STOP=1)
      __asm__("rim"); 														// Errata workaround (Enable interrupt)
      while(!(I2C->SR1 & I2C_SR1_RXNE)  &&  tout()); 		// test EV7, wait for RxNE
      *u8_DataBuffer = I2C->DR;                     		// Read Data byte
    }
  }
  /*--------------- All Data Received -----------------------*/
  while((I2C->CR2 & I2C_CR2_STOP)  &&  tout());     		// Wait until stop is performed (STOPF = 1)
  I2C->CR2 &=~I2C_CR2_POS;                          		// return POS to default state (POS=0)
}


/******************************************************************************
* Function name : ErrProc
* Description 	: Manage all I2C error detected by interrupt handler
* Input param 	: None
* Return 		    : None
* See also 		  : None
*******************************************************************************/
void ErrProc(void)
{
		// Clear Error Falg
    I2C->SR2= 0;
		// STOP=1, generate stop
	  I2C->CR2 |= 2;  
		// Disable Timout 
    TIM4_tout= 0;
		// Switch on LED3 for I2C Error detection
    //switch_on(LED3);
}

void TIM4_Init (void) {
  TIM4->ARR = 0x80;                // init timer 4 1ms inetrrupts
  TIM4->PSCR= 7;
  TIM4->IER = 1;
  TIM4->CR1 |= 1;
}


/* TCA9535 Setting Function */
void set_TCA9535(IO_Expander io_exp){
  
  // Set Timer for I2C Communication
  set_tout_ms(500);

  uint8_t input_set[1]={0xff};
  uint8_t output_set[1]={0x00};

  if (io_exp.io_set==1){ // Input Mode
    
    I2C_WriteRegister(io_exp.i2c_addr, CONFIGURATION_PORT_0, 1, input_set);
    I2C_WriteRegister(io_exp.i2c_addr, CONFIGURATION_PORT_1, 1, input_set);

  }else{ // Output Mode

    I2C_WriteRegister(io_exp.i2c_addr, CONFIGURATION_PORT_0, 1, output_set);
    I2C_WriteRegister(io_exp.i2c_addr, CONFIGURATION_PORT_1, 1, output_set);
    
  }

}


uint16_t read_io_expander(IO_Expander io_exp){
  uint8_t read_data[2];
  
  //  I2C_ReadRegister(u8 slave_address, u8 u8_regAddr, u8 u8_NumByteToRead, u8 *u8_DataBuffer)

  I2C_ReadRegister(io_exp.i2c_addr, INPUT_PORT_0, 1, read_data); // PORT0 Read

  I2C_ReadRegister(io_exp.i2c_addr, INPUT_PORT_1, 1, read_data+1); // PORT1 Read

  return (((uint16_t)read_data[1]<<8) | read_data[0]);
}


void write_io_expander(IO_Expander io_exp, uint16_t write_data){

  uint8_t io_exp_val[2]={ (uint8_t)write_data, (uint8_t)(write_data>>8) };
  
  I2C_WriteRegister(io_exp.i2c_addr, OUTPUT_PORT_0, 1, io_exp_val); // PORT0 Write

  I2C_WriteRegister(io_exp.i2c_addr, OUTPUT_PORT_1, 1, io_exp_val+1); // PORT1 Write
  

}

/* INA219 Setting Function */
void set_INA219(INA219 ina219_obj){
  /*
  < INA219 Configuration >
   - BUS Voltage Range : 32V FSR 
   - PGA(Shunt Voltage Only) : Gain => 1/8, Range : 320mV
   - BADC(Bus ADC Resolution) : Samples => 12bit, Conversion Time : 532us 
   - SADC(Shunt ADC Resolution) : Samples => 12bit, Conversion Time : 532us 
   - Operating Mode : Shunt and bus, continuous 

  */

  // Set Timer for I2C Communication
  set_tout_ms(500);

  // [1] Set Config register to take into account the settings above
  uint16_t config = INA219_CONFIG_BVOLTAGERANGE_32V |
                    INA219_CONFIG_GAIN_8_320MV | INA219_CONFIG_BADCRES_12BIT |
                    INA219_CONFIG_SADCRES_12BIT_1S_532US |
                    INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS;
  
  uint8_t config_val[2]={(uint8_t)(config>>8), (uint8_t)config};
  
  I2C_WriteRegister(ina219_obj.i2c_addr,INA219_REG_CONFIG, 2, config_val);


  // [2] Set Calibration register to 'Cal' calculated above
  uint8_t cali_val[2]={(uint8_t)(ina219_obj.Calibration_Value>>8), (uint8_t)(ina219_obj.Calibration_Value)};

  I2C_WriteRegister(ina219_obj.i2c_addr, INA219_REG_CALIBRATION, 2, cali_val);
}


float getCurrent_mA(INA219 ina219_obj){
  // [1] Set Calibration 
  //  Sometimes a sharp load will reset the INA219, which will reset the cal register,
  uint8_t cali_val[2]={(uint8_t)(ina219_obj.Calibration_Value>>8), (uint8_t)(ina219_obj.Calibration_Value)};

  I2C_WriteRegister(ina219_obj.i2c_addr, INA219_REG_CALIBRATION, 2, cali_val);

  // [2] Read Current Register
  uint8_t cur_data[2];

  I2C_ReadRegister(ina219_obj.i2c_addr, INA219_REG_CURRENT, 2, cur_data);

  uint16_t cur_read_val= (cur_data[0]<<8) | cur_data[1];

  //float ret=(cur_read_val * ina219_obj.Current_LSB_mA);
  // (3) Calc Current(mA)
  return cur_read_val;

}