#include "arduino.h"

#define LENGTH 100

bool started= false;    //True: Message is strated
bool ended  = false;    //True: Message is finished
char incomingByte;     //Variable to store the incoming byte
char msg[LENGTH];       //Message - array from 0 to LENGTH
byte index;             //Index of array
char letter;
String cool_word = "";
char return_char = '\0';

char xbee_recieve() {
    while (Serial.available() > 0) {
        incomingByte = Serial.read();
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
        //Serial.print(cool_word); //Use this to debug, typeof(cool_word) == String
        if (msg[0] != ' ') {
            return_char = msg[0];
        }
        cool_word = "";
        index = 0;
        msg[index] = '\0';
        started = false;
        ended = false;
        Serial.println("returning: ");
        Serial.println(return_char);
        return return_char;
    }
    else {
        return '\0';
    }
}
