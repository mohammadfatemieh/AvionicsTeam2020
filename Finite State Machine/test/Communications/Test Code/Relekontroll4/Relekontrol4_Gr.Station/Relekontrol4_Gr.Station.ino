#include "utilities.h"
#include "xbee_reciever.h"

void setup(){
    setupPins();
    Serial.begin(9600);
    xbee_Serial_init();
    while(!Serial);
    xbee_transmit("Ready to recieve data");
}

void loop() {
    char control = xbee_recieve();
    valveControl(control);
    ignition(control);
    statusCheck(control);
}
