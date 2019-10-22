#define BAUDRATE 115200

void setup() {
  // put your setup code here, to run once:
  Serial.begin(BAUDRATE);
  Serial5.begin(BAUDRATE);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (Serial.available()) {
    Serial5.write(Serial.read());
  }

  if (Serial5.available()) {
    Serial.write(Serial5.read());
  }

}
