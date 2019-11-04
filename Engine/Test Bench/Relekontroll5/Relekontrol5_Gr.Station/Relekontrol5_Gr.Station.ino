#include "states.h"
#include "xbee_GS.h"

int counter = 1;

void setup(){
    setupPins();
    Serial.begin(9600);
    xbee_Serial_init();
    while(!Serial);
    xbee_transmit("Ready to recieve data");
    Serial.println("Ready to send data");
}

void loop() {
//    if (counter >= 1000) {
//        xbee_transmit("I work!");
//        Serial.println("I work!");
//        counter = 1;
//    }
//    counter ++;
    char control = xbee_recieve();
    valveControl(control);
    ignition(control);
    statusCheck(control);
    delay(20);
}
