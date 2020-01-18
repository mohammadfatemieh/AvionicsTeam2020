//HC-12 messenger send/receive
//autor Tom Heylen tomtomheylen.com

void setup() {
  Serial.begin(9600);
}

void loop() {
  for (int i = 0; i < 11; i++) {
    Serial.println(i);
    delay(500);
  }
}
