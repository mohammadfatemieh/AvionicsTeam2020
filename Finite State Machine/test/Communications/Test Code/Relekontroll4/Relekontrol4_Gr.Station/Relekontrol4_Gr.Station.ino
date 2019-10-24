#include <SoftwareSerial.h>
#include "utilities.h"
#include "xbee_reciever.h"

SoftwareSerial xbee_Serial(2, 3); //RX, TX

void setup(){
    setupPins();
    Serial.begin(9600);
    xbee_Serial.begin(9600);
    while(!Serial);
    xbee_transmit("Ready to recieve data");
}

void loop() {
    volatile char control = xbee_recieve();
    valveControl(control);
    ignition(control);
    statusCheck(control);
}
