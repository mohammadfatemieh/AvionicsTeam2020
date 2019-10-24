#include "utilities.h"
#include "xbee_reciever.h"

int counter = 1;

void setup(){
    setupPins();
    Serial.begin(9600);
    xbee_Serial_init();
    while(!Serial);
    xbee_transmit("Ready to recieve data");
}

void loop() {
//    if (counter >= 20000) {
//        xbee_transmit("I_work!");
//        Serial.println("I_work!");
//        counter = 1;
//    }
//    counter ++;
    char control = xbee_recieve();
    valveControl(control);
    ignition(control);
    statusCheck(control);
    delay(20);
}
