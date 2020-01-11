#include <Arduino.h>
#define LENGTH_TEMP 4
#define LENGTH_GPS 10

#include <SoftwareSerial.h>

SoftwareSerial transmitSerial(2, 3); //RX, TX

//Constants:

//Variables:
String aString;
int aStringLen;

void setup() {
  //Start the serial communication
  Serial.begin(9600);
  transmitSerial.begin(9600); //Baud rate must be the same as is on xBee module
}

void loop() {
  while (Serial.available() > 0) {
    aString = Serial.readStringUntil('\n');       //read serial and save to aString
    aStringLen = aString.length();                //length of aString
    char charArr[aStringLen + 1] = { (char)0 };   //create char array
    aString.toCharArray(charArr, aStringLen);     //convert aString to char array

    switch (aStringLen-1) {
      case LENGTH_TEMP:
        transmitSerial.print('<');                        //indicate start of message
        for (int i = 0; i < aStringLen - 1; i++) {
          transmitSerial.print(charArr[i]);               //transmit char array
          Serial.print(charArr[i]);
        }
        transmitSerial.println('>');                      //indicate end of message
        Serial.println();
        break;
      case LENGTH_GPS:
        transmitSerial.print('#');                        //indicate start of message
        for (int i = 0; i < aStringLen - 1; i++) {
          transmitSerial.print(charArr[i]);               //transmit char array
          Serial.print(charArr[i]);
        }
        transmitSerial.println('#');                      //indicate end of message
        Serial.println();
        break;
      default:
        transmitSerial.print("Unknown data: ");
        transmitSerial.println(charArr);
        break;
    }
  }
}
