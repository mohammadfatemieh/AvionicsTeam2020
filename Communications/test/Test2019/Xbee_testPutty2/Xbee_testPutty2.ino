
void setup() {
  //Start the serial communication
  Serial.begin(9600); //Baud rate must be the same as is on xBee module
}

char letter;

void loop() {
  while (Serial.available() > 0) {
    letter = Serial.read();
    if (letter == '/n') {
        Serial.print('<');  //Starting symbol
        Serial.print(letter);//Value from 0 to 255
        Serial.println('>');//Ending symbol
    }
    delay(400);
  }
}
