#ifndef __STM8S_DEV_I2C_H
#define __STM8S_DEV_I2C_H

#include "stm8s.h"

/* < IO Expander Setting > */

typedef struct IO_Expander
{
  uint8_t device_no; // Io Expander Number to identify
  uint8_t io_set; // IO Setting to config Input or Output
  uint8_t i2c_addr; // device address

} IO_Expander;

#define IO_EXP_ADDR1 0x20

/* < INA219 Setting > */
// REF1 : 0.04096 is an internal fixed value used to ensure scaling is maintained properly

typedef struct INA219
{
  uint8_t device_no; // device index
  uint8_t i2c_addr; // device i2c slave address

  uint8_t VBUS_Max; // VBUS Max Voltage(V) : 32V or 16V 
  float R_Shunt; // Shunt Resistor(ohm)
  uint32_t ADC_RES; // ADC Resolution (2^bit)
  float Max_possible_Curr; // Max Possible Current  
  float Max_expected_Curr; // Max Expected Current

  float Current_LSB_mA; // Current LSB

  uint16_t Calibration_Value; // Calibration Value
  
  float currentDivider_mA; // To obtain a value in amperes the Current register value is multiplied by the programmed Current_LSB
  float powerMultiplier_mW; // Power Register content is multiplied by Power LSB which is 20 times the Current_LSB for a power value in watts.
  

} INA219;

#define INA219_ADDR 0x40

void set_INA219(INA219 ina219_obj); // Set Calibration Register and Config Register
float getCurrent_mA(INA219 ina219_obj);

// Register Address
#define INA219_REG_CONFIG (0x00) // config register address
#define INA219_REG_SHUNTVOLTAGE (0x01) // shunt voltage register 
#define INA219_REG_BUSVOLTAGE (0x02) // bus voltage register
#define INA219_REG_POWER (0x03) // power register
#define INA219_REG_CURRENT (0x04) // current register 
#define INA219_REG_CALIBRATION (0x05) // calibration register

// Mask bit
#define INA219_CONFIG_RESET (0x8000) // Reset Bit
#define INA219_CONFIG_BVOLTAGERANGE_MASK (0x2000) // Bus Voltage Range Mask
#define INA219_CONFIG_GAIN_MASK (0x1800) // Gain Mask
#define INA219_CONFIG_BADCRES_MASK (0x0780) // mask for bus ADC resolution bits **/
#define INA219_CONFIG_SADCRES_MASK (0x0078) // Shunt ADC Resolution and Averaging Mask 
#define INA219_CONFIG_MODE_MASK (0x0007) // Operating Mode Mask

// Config bit
/** bus voltage range values **/
enum {
  INA219_CONFIG_BVOLTAGERANGE_16V = (0x0000), // 0-16V Range
  INA219_CONFIG_BVOLTAGERANGE_32V = (0x2000), // 0-32V Range
};

/** values for gain bits **/
enum {
  INA219_CONFIG_GAIN_1_40MV = (0x0000),  // Gain 1, 40mV Range
  INA219_CONFIG_GAIN_2_80MV = (0x0800),  // Gain 2, 80mV Range
  INA219_CONFIG_GAIN_4_160MV = (0x1000), // Gain 4, 160mV Range
  INA219_CONFIG_GAIN_8_320MV = (0x1800), // Gain 8, 320mV Range
};

/** values for bus ADC resolution **/
enum {
  INA219_CONFIG_BADCRES_9BIT = (0x0000),  // 9-bit bus res = 0..511
  INA219_CONFIG_BADCRES_10BIT = (0x0080), // 10-bit bus res = 0..1023
  INA219_CONFIG_BADCRES_11BIT = (0x0100), // 11-bit bus res = 0..2047
  INA219_CONFIG_BADCRES_12BIT = (0x0180), // 12-bit bus res = 0..4097
  INA219_CONFIG_BADCRES_12BIT_2S_1060US =(0x0480), // 2 x 12-bit bus samples averaged together
  INA219_CONFIG_BADCRES_12BIT_4S_2130US = (0x0500), // 4 x 12-bit bus samples averaged together
  INA219_CONFIG_BADCRES_12BIT_8S_4260US = (0x0580), // 8 x 12-bit bus samples averaged together
  INA219_CONFIG_BADCRES_12BIT_16S_8510US = (0x0600), // 16 x 12-bit bus samples averaged together
  INA219_CONFIG_BADCRES_12BIT_32S_17MS = (0x0680), // 32 x 12-bit bus samples averaged together
  INA219_CONFIG_BADCRES_12BIT_64S_34MS = (0x0700), // 64 x 12-bit bus samples averaged together
  INA219_CONFIG_BADCRES_12BIT_128S_69MS = (0x0780), // 128 x 12-bit bus samples averaged together

};

/** values for shunt ADC resolution **/
enum {
  INA219_CONFIG_SADCRES_9BIT_1S_84US = (0x0000),   // 1 x 9-bit shunt sample
  INA219_CONFIG_SADCRES_10BIT_1S_148US = (0x0008), // 1 x 10-bit shunt sample
  INA219_CONFIG_SADCRES_11BIT_1S_276US = (0x0010), // 1 x 11-bit shunt sample
  INA219_CONFIG_SADCRES_12BIT_1S_532US = (0x0018), // 1 x 12-bit shunt sample
  INA219_CONFIG_SADCRES_12BIT_2S_1060US = (0x0048), // 2 x 12-bit shunt samples averaged together
  INA219_CONFIG_SADCRES_12BIT_4S_2130US = (0x0050), // 4 x 12-bit shunt samples averaged together
  INA219_CONFIG_SADCRES_12BIT_8S_4260US = (0x0058), // 8 x 12-bit shunt samples averaged together
  INA219_CONFIG_SADCRES_12BIT_16S_8510US = (0x0060), // 16 x 12-bit shunt samples averaged together
  INA219_CONFIG_SADCRES_12BIT_32S_17MS = (0x0068), // 32 x 12-bit shunt samples averaged together
  INA219_CONFIG_SADCRES_12BIT_64S_34MS = (0x0070), // 64 x 12-bit shunt samples averaged together
  INA219_CONFIG_SADCRES_12BIT_128S_69MS = (0x0078), // 128 x 12-bit shunt samples averaged together
};

/** values for operating mode **/
enum {
  INA219_CONFIG_MODE_POWERDOWN = 0x00,       /*< power down */
  INA219_CONFIG_MODE_SVOLT_TRIGGERED = 0x01, /*< shunt voltage triggered */
  INA219_CONFIG_MODE_BVOLT_TRIGGERED = 0x02, /*< bus voltage triggered */
  INA219_CONFIG_MODE_SANDBVOLT_TRIGGERED = 0x03, /*< shunt and bus voltage triggered */
  INA219_CONFIG_MODE_ADCOFF = 0x04, /*< ADC off */
  INA219_CONFIG_MODE_SVOLT_CONTINUOUS = 0x05, /*< shunt voltage continuous */
  INA219_CONFIG_MODE_BVOLT_CONTINUOUS = 0x06, /*< bus voltage continuous */
  INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS = 0x07, /*< shunt and bus voltage continuous */
};


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

void I2C_ReadRegister(u8 slave_address, u8 u8_regAddr, u8 u8_NumByteToRead, u8 *u8_DataBuffer);

void I2C_WriteRegister(u8 slave_address, u8 u8_regAddr, u8 u8_NumByteToWrite, u8 *u8_DataBuffer);

uint8_t I2C_ReadByte(uint8_t slaveAddr);
uint16_t I2C_Read_2Bytes(uint8_t slave_address);

void I2C_RandomRead(u8 u8_NumByteToRead, u8 *u8_DataBuffer, u8 slave_address);


void ErrProc(void);
#endif