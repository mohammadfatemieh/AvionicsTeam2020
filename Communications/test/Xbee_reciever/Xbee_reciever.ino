#define LENGTH 100

const int ledPin = 3; //Led to Arduino pin 3 (PWM)

bool started= false;//True: Message is strated
bool ended  = false;//True: Message is finished
char incomingByte ; //Variable to store the incoming byte
char msg[LENGTH];    //Message - array from 0 to 2 (3 values - PWM - e.g. 240)
byte index;     //Index of array
char letter;
String cool_word = "";

void setup() {
  //Start the serial communication
  Serial.begin(9600); //Baud rate must be the same as is on xBee module
  pinMode(ledPin, OUTPUT);
}


void loop() {
   xbee_recieve();
}

void xbee_recieve() {
    while (Serial.available()>0) {
        //Read the incoming byte
        incomingByte = Serial.read();
        //Start the message when the '<' symbol is received
        if(incomingByte == '<') {
            started = true;
            index = 0;
            //msg[index] = '\0'; // Throw away any incomplete packet
        }
        //End the message when the '>' symbol is received
        else if(incomingByte == '>') {
            ended = true;
            break; // Done reading - exit from while loop!
        }
        //Read the message!
        else {
            if(index < LENGTH) {  // Make sure there is roof
                msg[index] = incomingByte; // Add char to array
                cool_word += msg[index];
                index++;
                //msg[index] = '\0'; // Add NULL to end
            }
        }
    }

    if (started && ended) {
        Serial.print(cool_word); //Only for debugging
        cool_word = "";
        index = 0;
        msg[index] = '\0';
        started = false;
        ended = false;
    }
}
