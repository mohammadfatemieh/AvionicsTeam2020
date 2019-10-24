#include "arduino.h"
#include <SoftwareSerial.h>

#define LENGTH 100

bool started= false;    //True: Message is strated
bool ended  = false;    //True: Message is finished
char incomingByte;     //Variable to store the incoming byte
char msg[LENGTH];       //Message - array from 0 to LENGTH
byte index;             //Index of array
char letter;
String cool_word = "";
char return_char = '\0';

SoftwareSerial xbee_Serial(2, 3); //RX, TX

void xbee_Serial_init() {
  xbee_Serial.begin(9600);
}

int xbee_Serial_available() {
  return xbee_Serial.available();
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
        //xbee_transmit("returning: ");
        //xbee_transmit(return_char);
        return return_char;
    }
    else {
        return '\0';
    }
}

void xbee_transmit(String cool_word) {
    xbee_Serial.print('<');
    xbee_Serial.print(cool_word);
    xbee_Serial.println('>');
}

void xbee_transmit(int value) {
    xbee_Serial.print('<');
    xbee_Serial.print(value);
    xbee_Serial.println('>');
}

void xbee_transmit(String cool_word, int value) {
    xbee_Serial.print('<');
    xbee_Serial.print(cool_word);
    xbee_Serial.print(" ");
    xbee_Serial.print(value);
    xbee_Serial.println('>');
}
