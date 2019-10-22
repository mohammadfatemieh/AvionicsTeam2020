
void setup() {
  //Start the serial communication
  Serial.begin(9600); //Baud rate must be the same as is on xBee module
}

char letter;

void loop() {
//  while (Serial.available() > 0) {
//    letter = Serial.read();
//    Serial.println(letter); //printing the letter
//    delay(200);
//  }
    for (int i = 0; i < 10; i++) {
        Serial.println(i);
        delay(1000);
    }
}
