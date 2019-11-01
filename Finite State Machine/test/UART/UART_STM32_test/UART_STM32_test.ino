/* TESTING IF UART ON STM32 WORKS BY RECEIVING A UART MESSAGE
 * AND PRINTING IT TO THE SERIAL MONITOR.
 */

#include <SoftwareSerial.h>

// Baudrate of UART
#define BAUD 9600

// Define software UART pins
const byte RxPin = 2;
const byte TxPin = 3;

// set up a new serial object
SoftwareSerial uart (RxPin, TxPin);

void setup() {
  // Set up UART pins
  pinMode(RxPin, INPUT);
  pinMode(TxPin, OUTPUT);
  
  // Begin UART connection
  uart.begin(BAUD);

  // Begin Serial Monitor
  Serial.begin(BAUD);
}

void loop() {
  
  // Check for received UART message
  if(uart.available() > 0)
  {
    // Read UART 
    char c = uart.read();
  
    // Write to Serial Monitor
    Serial.print("UART: ");
    Serial.println(c);
  }
}
