#include <Wire.h>

#define SLAVE_ADDRESS 0x33  // Same as in STM8 code

uint8_t receivedData = 0x00;  // Store received data
uint8_t responseData = 0xCD;  // Example response data

void setup() {
    Wire.begin(SLAVE_ADDRESS); // Join I2C bus with slave address
    Wire.onReceive(receiveEvent);
    Wire.onRequest(requestEvent);
    Serial.begin(9600);
}

void loop() {
    delay(100);  // Just to keep the loop running
}

// Function to handle received data from STM8
void receiveEvent(int numBytes) {
    if (Wire.available()) {
        receivedData = Wire.read();  // Read the byte sent by STM8
        Serial.print("Received: 0x");
        Serial.println(receivedData, HEX);
    }
}

// Function to handle master read request
void requestEvent() {
    Wire.write(responseData);  // Send response data to STM8
    Serial.println("Sent: 0xCD");
}