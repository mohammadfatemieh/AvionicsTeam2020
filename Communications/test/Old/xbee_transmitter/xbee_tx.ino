#include "XBee.h"

#define NUM_FLAGS 10
#define NUM_SENS 5

float sensors[NUM_SENS] = {500, 234, 121, 111, 255};
bool flags[NUM_FLAGS] = {true, true, true, false, true, false, true, true, false, true};

// Må caste sensors til void*
XBee xbee((void *)sensors, flags, NUM_SENS*sizeof(sensors[0]), NUM_FLAGS);


void setup() {
  
}

int i = 0;
void loop() {
   sensors[1]++;
   xbee.transmit();
   delay(1000);

    
  
}
