#define LENGTH 100 // With new line added automatically
#include <SoftwareSerial.h>

SoftwareSerial mySerial(2, 3); //RX, TX

//Constants:
const int potPin = A0; //Pot at Arduino A0 pin
//Variables:
int value ; //Value from pot


void setup() {
    //Start the serial communication
    Serial.begin(9600);
    mySerial.begin(9600); //Baud rate must be the same as is on xBee module
}

char a_word[LENGTH+1] = { (char)0 };
int index;
char letter;

void loop() {
    while (Serial.available() > 0) {
        letter = Serial.read();
        a_word[index] = letter;
        index++;

        if (letter == '\n') {
            mySerial.print('<');
            for (int i = 0; i < index-1; i++) {
                mySerial.print(a_word[i]);
            }
            mySerial.println('>');
            index = 0;
            for (int i = 0; i < sizeof(a_word);i++) {
                a_word[i] = (char)0;
            }
        }
    }
}
