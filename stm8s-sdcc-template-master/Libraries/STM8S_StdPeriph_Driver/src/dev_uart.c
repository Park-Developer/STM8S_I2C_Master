#include "dev_uart.h"
#include <string.h> 
#include <stdio.h>

void uart1_config(void){
    GPIO_Init(GPIOD, GPIO_PIN_5, GPIO_MODE_OUT_PP_HIGH_FAST);
    GPIO_Init(GPIOD, GPIO_PIN_6, GPIO_MODE_IN_PU_NO_IT);
  
    // UART Init
    UART1_Init(115200, UART1_WORDLENGTH_8D, UART1_STOPBITS_1,  UART1_PARITY_NO, UART1_SYNCMODE_CLOCK_DISABLE, UART1_MODE_TXRX_ENABLE);
    UART1_ITConfig(UART1_IT_RXNE_OR, ENABLE);
    uart_write("hello world!! \r\n");
    UART1_Cmd(ENABLE);
}

int uart_write(const char *str) {
    char i;
    for(i = 0; i < strlen(str); i++) {
        while(!(UART1->SR & UART1_SR_TXE)); // !Transmit data register empty
        UART1->DR = str[i];
    }
    return(i); // Bytes sent
}

// Function to send a single byte over UART
void UART1_SendByte(uint8_t byte) {
    while (!(UART1->SR & UART1_SR_TXE)); // Wait until TXE (Transmit Data Register Empty)
    UART1->DR = byte;                   // Write byte to UART data register
}

// Function to send a string over UART
void UART1_SendString(const char *str) {
    while (*str) {
        UART1_SendByte(*str++);
    }
}

// Function to send a register value in hexadecimal format
void UART1_SendRegisterValue(const char *regName, uint8_t regValue) {
    char buffer[16]; // Temporary buffer for formatted output
    UART1_SendString(regName);          // Send register name
    UART1_SendString(": 0x");           // Send "0x" prefix

    // Format the register value as a 2-digit hexadecimal string
    sprintf(buffer, "%02X", regValue);
    UART1_SendString(buffer);           // Send the formatted value

    UART1_SendString("\r\n");           // Newline for readability
}


void UART1_Send16(uint16_t value) {
    char buffer[6]; // Buffer for ADC value string
    sprintf(buffer, "%u", value); // Convert ADC value to string
    UART1_SendString(buffer);
}
