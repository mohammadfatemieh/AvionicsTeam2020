#include <SoftwareSerial.h>
#define LENGTH 12

SoftwareSerial xbee_Serial(2, 3); //RX, TX


//Variables:
char char_array[LENGTH + 1] = { (char)0 };
bool started = false;   //True: Message is strated
bool ended  = false;    //True: Message is finished
char incomingByte;     //Variable to store the incoming byte
String incomingString;
char msg[LENGTH + 1];     //Message - array from 0 to LENGTH
byte index;             //Index of array
char letter;
String cool_word = "";

void setup() {
  Serial.begin(9600); //Baud rate must be the same as is on xBee module
  xbee_Serial.begin(9600);
}

void xbee_recieve_and_print();
void xbee_transmit(String word);

void loop() {
  //xbee_transmit();
  xbee_recieve_and_print();
  delay(100);
}

void xbee_transmit() {
  while (Serial.available() > 0) {
    letter = Serial.read();
    char_array[index] = letter;
    index++;
    for (int i = 0; i < 1; i++) {
      if (letter == '\n') {
        Serial.print("Sending: ");
        Serial.println(char_array);
        xbee_Serial.print('<');
        for (int i = 0; i < index - 1; i++) {
          xbee_Serial.print(char_array[i]);
        }
        xbee_Serial.println('>');
        index = 0;
        for (int i = 0; i < sizeof(char_array); i++) {
          char_array[i] = (char)0;
        }
      }
      delay(20);
    }
  }
}

void xbee_recieve_and_print() {
  while (xbee_Serial.available() > 0) {
    /*incomingByte = xbee_Serial.read();
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
      Serial.println(cool_word); //Use this to debug, typeof(cool_word) == String
      cool_word = "";
      index = 0;
      for (int i = 0; i < sizeof(msg); i++) {
        msg[i] = (char)0;
      }
      started = false;
      ended = false;

      }*/
    incomingString = xbee_Serial.readString();
    //incomingString.remove(0,1);
    //incomingString.remove(incomingString.length()-3);
    Serial.println(incomingString);
  }
}
