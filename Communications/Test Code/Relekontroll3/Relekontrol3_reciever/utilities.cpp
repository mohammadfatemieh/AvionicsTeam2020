#include "arduino.h"

#define outDumpPin 13
#define outFillPin 12
#define outArmPin 11
#define outPyroPin 10
#define outOxPin 9
#define outAirPin 8

int armingStatus = 0;
int fillStatus = 0;
int dumpStatus = 0;
int airingStatus = 0;

int wait = 1000;
int oxdelay = 500;

void setupPins() {
    pinMode(outDumpPin, OUTPUT);
    digitalWrite(outDumpPin, HIGH);
    pinMode(outFillPin, OUTPUT);
    digitalWrite(outFillPin, HIGH);
    pinMode(outArmPin, OUTPUT);
    digitalWrite(outArmPin, HIGH);
    pinMode(outPyroPin, OUTPUT);
    digitalWrite(outPyroPin, HIGH);
    pinMode(outOxPin, OUTPUT);
    digitalWrite(outOxPin, HIGH);
    pinMode(outAirPin, OUTPUT);
    digitalWrite(outAirPin, HIGH);
}

void valveControl(char valve){
  switch(valve){
    case 'D':
      if(fillStatus == 0){
        digitalWrite(outDumpPin, LOW);
        dumpStatus = 1;
        Serial.println("Dump valve is now OPEN");
      } else {
        Serial.println("Filling valve must be closed while dumping.");
      }
      break;

    case 'd':
      digitalWrite(outDumpPin, HIGH);
      dumpStatus = 0;
      Serial.println("Dump valve is now CLOSED");
      break;

    case 'F':
      if(dumpStatus == 0){
        digitalWrite(outFillPin, LOW);
        fillStatus = 1;
        Serial.println("Fill valve is now OPEN");
      } else {
        Serial.println("Dump valve must be closed while filling.");
      }
      break;

    case 'f':
      digitalWrite(outFillPin, HIGH);
      fillStatus = 0;
      Serial.println("Fill valve is now CLOSED");
      break;

    case 'I':
      digitalWrite(outAirPin, LOW);
      airingStatus = 1;
      Serial.println("Airing valve is now CLOSED");
      break;

    case 'i':
      digitalWrite(outAirPin, HIGH);
      airingStatus = 0;
      Serial.println("Airing valve is now OPEN");
      break;
  }
}

void ignition(char order){
  switch(order){
    case 'A':
      if((dumpStatus == 0) && (fillStatus == 0) && (airingStatus == 0)){
        Serial.println("Caution: Rocket is about to arm. Confirm with Y/N");
        char confirmArm = ' ';
        while(true){
          confirmArm = Serial.read();
          if(confirmArm == 'Y'){
            digitalWrite(outArmPin, LOW);
            armingStatus = 1;
            Serial.println("WARNING! WARNING! WARNING!");
            Serial.println("ROCKET IS NOW ARMED AND READY TO FIRE. COMMAND a TO DISARM");
            break;
          }
          else if (confirmArm == 'N') {
            Serial.println("Arming cancelled");
            break;
          }
          else if (Serial.available()){
             Serial.println("To Arm: Type 'Y', otherwise 'N'");
          }
        }
      }
      else {
          Serial.println("No valves open when arming!");
      }
      break;
    case 'a':
      digitalWrite(outArmPin, HIGH);
      armingStatus = 0;
      Serial.println("Rocket is not armed, and will not fire");
      break;
    case 'O':
      if(armingStatus == 1){
        Serial.println("Rocket will fire in 5 seconds");
        delay(5 * wait);
        digitalWrite(outPyroPin, LOW);//IGNITES PYRO
        delay(oxdelay);
        digitalWrite(outOxPin, LOW); //ACTIVATES RUN VALVE
        Serial.println("Rocket has fired");
      }
      break;
    case 'o':
      digitalWrite(outPyroPin, HIGH);
      digitalWrite(outOxPin, HIGH);
      digitalWrite(outArmPin, HIGH);
      Serial.println("Pyro, arm and ox disabled");
      break;
  }
}

void statusCheck(char check){
    if (check == 'S'){
        Serial.println("\nCurrent status is:\n");
        Serial.print("Dumpstatus is: ");
        Serial.println(dumpStatus);
        Serial.print("Airing status is: ");
        Serial.println(airingStatus);
        Serial.print("Filling status is: ");
        Serial.println(fillStatus);
        Serial.print("Arming status is: ");
        Serial.println(armingStatus);
        Serial.println();
    }
}
