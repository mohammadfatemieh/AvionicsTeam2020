#define LENGTH 100 // With new line added automatically


//Variables:
char char_array[LENGTH+1] = { (char)0 };
int index;
char letter;


void setup() {
    Serial.begin(9600); //Baud rate must be the same as is on xBee module
}


void xbee_transmit();

void loop() {
    xbee_transmit();
}

void xbee_transmit() {
    while (Serial.available() > 0) {
        letter = Serial.read();
        char_array[index] = letter;
        index++;
        if (letter == '\n') {
            Serial.print('<');
            for (int i = 0; i < index-1; i++) {
                Serial.print(char_array[i]);
            }
            Serial.println('>');
            index = 0;
            for (int i = 0; i < sizeof(char_array);i++) {
                char_array[i] = (char)0;
            }
        }
    }
}
