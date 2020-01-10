int outDumpPin = 13;
int outFillPin = 12;
int outArmPin = 11;
int outPyroPin = 10;
int outOxPin = 9;
int outAirPin = 8;

int armingStatus = 0;
int fillStatus = 0;
int dumpStatus = 0;
int airingStatus = 0;

int wait = 1000;
int oxdelay = 500;




void setup(){
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

  Serial.begin(9600);
  while(!Serial){

  }
  Serial.println("Ready to recieve data");
  Serial.println();
}

void valveControl(char valve){
  switch(valve){
    case 'D':
      if(fillStatus == 0){
        digitalWrite(outDumpPin, LOW);
        dumpStatus = 1;
        Serial.print("Dump valve is now OPEN");
        Serial.println();
      }
      break;

    case 'd':
      digitalWrite(outDumpPin, HIGH);
      dumpStatus = 0;
      Serial.print("Dump valve is now CLOSED");
      Serial.println();
      break;

    case 'F':
      if(dumpStatus == 0){
        digitalWrite(outFillPin, LOW);
        fillStatus = 1;
        Serial.print("Fill valve is now OPEN");
        Serial.println();
      }
      break;

    case 'f':
      digitalWrite(outFillPin, HIGH);
      fillStatus = 0;
      Serial.print("Fill valve is now CLOSED");
      Serial.println();
      break;

    case 'I':
      digitalWrite(outAirPin, LOW);
      airingStatus = 1;
      Serial.print("Airing valve is now OPEN");
      Serial.println();
      break;

    case 'i':
      digitalWrite(outAirPin, HIGH);
      airingStatus = 0;
      Serial.print("Airing valve is now CLOSED");
      Serial.println();
      break;
  }
}


void ignition(char order){
  switch(order){
    case 'A':
      if((dumpStatus == 0) && (fillStatus == 0)){
        Serial.println("Caution: Rocket is about to arm. Confirm with Y/[ANY]");
        while(!Serial.available()){
          //Wait for input
        }
        char confirmArm = Serial.read();
        if(confirmArm == 'Y'){
          digitalWrite(outArmPin, LOW);
          armingStatus = 1;
          Serial.print("WARNING! WARNING! WARNING!");
          Serial.println();
          Serial.print("ROCKET IS NOW ARMED AND READY TO FIRE. COMMAND a TO DISARM");
          Serial.println();
        }
        else{
          Serial.println("Arming cancelled");
        }
      }
      break;
    case 'a':
      digitalWrite(outArmPin, HIGH);
      armingStatus = 0;
      Serial.print("Rocket is not armed, and will not fire");
      Serial.println();
      break;
    case 'O':
      if(armingStatus == 1){
        Serial.print("Rocket will fire in 5 seconds");
        Serial.println();
        delay(5 * wait);
        digitalWrite(outPyroPin, LOW);//IGNITES PYRO
        delay(oxdelay);
        digitalWrite(outOxPin, LOW); //ACTIVATES RUN VALVE
        Serial.print("Rocket has fired");
        Serial.println();
      }
      break;
    case 'o':
      digitalWrite(outPyroPin, HIGH);
      digitalWrite(outOxPin, HIGH);
      digitalWrite(outArmPin, HIGH);
      Serial.print("Pyro, arm and ox disabled");
      Serial.println();
      break;
  }
}

void statusCheck(char check){
  if (check == 'S'){
    Serial.println();
    Serial.println("Current status is: ");
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


void loop() {
  // put your main code here, to run repeatedly:
  String inData;
  while(Serial.available()){
    inData = Serial.readString();
    inData.trim();
  }
  char control = inData[0];
  //char control = Serial.read();
  valveControl(control);
  ignition(control);
  statusCheck(control);
  delay(50);
}
