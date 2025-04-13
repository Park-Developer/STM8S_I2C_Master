/*
simulation code for INA219
*/
#include <Wire.h>

#define SLAVE_ADDRESS 0x08  // Same as in STM8 code

volatile uint8_t regAddr = 0;
volatile uint16_t receivedData = 0;
volatile uint8_t msb=0;
volatile uint8_t lsb=0; 
uint8_t responseData1 = 0xAB;  // Example response data
uint8_t responseData2 = 0xCD;  // Example response data
void setup() {
    Wire.begin(SLAVE_ADDRESS); // Join I2C bus with slave address
    Wire.onReceive(receiveEvent);
    Wire.onRequest(requestEvent);
    Serial.begin(9600);
}

void loop() {

  //delay(1000);  // Just to keep the loop running
}

// Function to handle received data from STM8
void receiveEvent(int howMany) {
  Serial.print("Reeceive!");
  if (howMany < 3) return; // Need at least 1 register + 2 bytes

  regAddr = Wire.read(); // First byte = register address
  msb = Wire.read();
  lsb = Wire.read();

  receivedData = (msb << 8) | lsb;
  Serial.print("Received data: 0x");
  Serial.println(receivedData, HEX);

  Serial.print("To register: 0x");
  Serial.println(regAddr, HEX);
}

// Function to handle master read request( send 2 byte )
void requestEvent() {

    Wire.write(responseData1);  // Send response data to STM8
    Wire.write(responseData2);  // Send response data to STM8

    
    Serial.println("Sent:");
}