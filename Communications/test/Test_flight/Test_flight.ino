#include <SoftwareSerial.h>
#include <Adafruit_MPL3115A2.h>

Adafruit_MPL3115A2 baro = Adafruit_MPL3115A2();

SoftwareSerial mySerial(2, 3); //RX, TX

int counter;
float values[3];

void setup() {
  Serial.begin(9600);
  mySerial.begin(9600);
}

void loop() { 
  if(counter > 50) {
    counter = 0;
    value = sense();
    transmitt(value);
  }
  
  if(Serial.available() > 0) {
    String input = Serial.readString();
    mySerial.println(input);
    Serial.print(input);
  }
  delay(20);
  counter ++;
}

void transmitt(float value) {
  mySerial.println(value);
}

void sense() {
  if (! baro.begin()) {
    Serial.println("Couldnt find sensor");
    return;
  }
  values[0] = (baro.getPressure())/3377;
  float pascals = baro.getPressure();
  Serial.print(pascals/3377); Serial.println(" Inches (Hg)");
 
  float altm = baro.getAltitude();
  Serial.print(altm); Serial.println(" meters");
 
  float tempC = baro.getTemperature();
  Serial.print(tempC); Serial.println("*C");
}
