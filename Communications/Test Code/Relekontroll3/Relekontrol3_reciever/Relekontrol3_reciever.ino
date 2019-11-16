#include "utilities.h"
#include "xbee_reciever.h"

void setup(){
    setupPins();
    Serial.begin(9600);
    while(!Serial);
    Serial.println("Ready to recieve data");
    Serial.println();
}

void loop() {
    /*
    String inData;
    while(Serial.available()) {  //Busy wait
        inData = Serial.readString();
        inData.trim();
    }
    char control = inData[0];
    */

    volatile char control = xbee_recieve();
    valveControl(control);
    ignition(control);
    statusCheck(control);
}
