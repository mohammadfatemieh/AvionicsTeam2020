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
    char charArr[aStringLen+1] = { (char)0 };     //create char array
    aString.toCharArray(charArr,aStringLen);      //convert aString to char array

//      switch (var) {
//        case label1:
//          // statements
//          break;
//        case label2:
//          // statements
//          break;
//        default:
//          // statements
//          break;
//      }
      
      transmitSerial.print('<');                        //indicate start of message
      for (int i = 0; i < aStringLen - 1; i++) {  
        transmitSerial.print(charArr[i]);               //transmit char array
      }
      transmitSerial.println('>');                      //indicate end of message
  }
}
