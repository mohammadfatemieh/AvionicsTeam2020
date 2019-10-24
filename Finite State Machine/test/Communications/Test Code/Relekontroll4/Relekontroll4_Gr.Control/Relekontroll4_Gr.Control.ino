#include <SoftwareSerial.h>
#define LENGTH 100 // With new line added automatically

SoftwareSerial xbee_Serial(2, 3); //RX, TX

//Variables:
char char_array[LENGTH+1] = { (char)0 };
int index;
char letter;


void setup() {
    Serial.begin(9600); //Baud rate must be the same as is on xBee module
    xbee_Serial.begin(9600);
}


void xbee_transmit();
void xbee_recieve();
void xbee_transmit(String word);

void loop() {
    xbee_transmit();
    xbee_recieve();
}

void xbee_transmit() {
    while (Serial.available() > 0) {
        letter = Serial.read();
        char_array[index] = letter;
        index++;
        if (letter == '\n') {
            xbee_Serial.print('<');
            for (int i = 0; i < index-1; i++) {
                xbee_Serial.print(char_array[i]);
            }
            xbee_Serial.println('>');
            index = 0;
            for (int i = 0; i < sizeof(char_array);i++) {
                char_array[i] = (char)0;
            }
        }
    }
}

void xbee_transmit(String word) {
    xbee_Serial.print('<');
    xbee_Serial.print(word);
    xbee_Serial.println('>');
}

char xbee_recieve() {
    while (xbee_Serial.available() > 0) {
        incomingByte = xbee_Serial.read();
        if(incomingByte == '<') {
            started = true;
            index = 0;
        }
        else if(incomingByte == '>') {
            ended = true;
            break;
        }
        else {
            if(index < LENGTH) {
                msg[index] = incomingByte;
                cool_word += msg[index];
                index++;
            }
        }
    }
    if (started && ended) {
        Serial.print(cool_word); //Use this to debug, typeof(cool_word) == String
        if (msg[0] != ' ') {
            return_char = msg[0];
        }
        cool_word = "";
        index = 0;
        msg[index] = '\0';
        started = false;
        ended = false;
        return return_char;
    }
    else {
        return '\0';
    }
}
