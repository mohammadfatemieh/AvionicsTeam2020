#include "xbee_tx.h"
#include <Arduino.h>


XBee::XBee(void* sensors, const uint8_t num_sens_bytes) 
        : num_sens_bytes(num_sens_bytes), timer(millis()), package_number(0), sensors((uint8_t*)sensors) {
         
          //Activating serial communication
          Serial5.begin(115200);
          Serial5.setTimeout(25);

          //Resetting module
          reset();

          //Setting up sleep pin
          //pinMode(SLEEP_PIN, OUTPUT);
          //digitalWrite(SLEEP_PIN, LOW);     
          Serial.println("XBee initialized");
    }


void XBee::reset(void) {
    pinMode(RESET_PIN, OUTPUT);
    digitalWrite(RESET_PIN, LOW);
    delay(1);
    pinMode(RESET_PIN, INPUT);
}

void XBee::transmit(void) {
    
    if(millis()-timer > TIMER_DELAY) {
        Serial5.write('<');
        Serial5.write((uint8_t*)&(++package_number), sizeof(package_number));
        Serial5.write(sensors, num_sens_bytes);
        Serial5.write('>');
        timer = millis();
    }
}


void XBee::sleep(void) {
    digitalWrite(SLEEP_PIN, HIGH);
}

void XBee::wake_up(void) {
    digitalWrite(SLEEP_PIN, LOW);
}



