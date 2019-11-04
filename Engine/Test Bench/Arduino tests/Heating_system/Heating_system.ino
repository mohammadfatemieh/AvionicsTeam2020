#define HEAT1_PIN 6
#define HEAT2_PIN 7


void setup() {
    // put your setup code here, to run once:
    pinMode(HEAT1_PIN, OUTPUT);
    digitalWrite(HEAT1_PIN, HIGH);
    pinMode(HEAT2_PIN, OUTPUT);
    digitalWrite(HEAT2_PIN, HIGH);
    Serial.begin(9600);
}

void loop() {
    if (Serial.available()) {
        char choice = Serial.read();
        Serial.print(choice);
        if (choice == 'a') {
            digitalWrite(HEAT1_PIN, LOW);
        } else if (choice == 'b') {
            digitalWrite(HEAT1_PIN, HIGH);
        } else if (choice == 'c') {
            digitalWrite(HEAT2_PIN, LOW);
        } else if (choice == 'd') {
            digitalWrite(HEAT2_PIN, HIGH);
        }
    }
}
