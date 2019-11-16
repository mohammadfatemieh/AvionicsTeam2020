
#include <SoftwareSerial.h>

SoftwareSerial mySerial(2, 3); //RX, TX

void setup() {
  Serial.begin(9600);
  mySerial.begin(9600);
}

void loop() {
   
  if(Serial.available() > 0){//Read from serial monitor and send over HC-12
    String input = Serial.readString();
    mySerial.println(input);
    Serial.print(input);
  }
 
//  if(mySerial.available() > 1){//Read from HC-12 and send to serial monitor
//    String input = mySerial.readString();
//    Serial.println(input);    
//  }

if(mySerial.available() > 1){//Read from HC-12 and send to serial monitor
    char input = mySerial.read();
    Serial.print(input);
  }
  delay(20);

}
