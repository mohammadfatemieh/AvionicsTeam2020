#define LENGTH 100

bool started= false;    //True: Message is strated
bool ended  = false;    //True: Message is finished
char incomingByte;     //Variable to store the incoming byte
char msg[LENGTH];       //Message - array from 0 to LENGTH
byte index;             //Index of array
char letter;
String cool_word = "";

void setup() {
  Serial.begin(9600); //Baud rate must be the same as is on xBee module
}


void loop() {
   xbee_recieve();
}

void xbee_recieve() {
    while (Serial.available() > 0) {
        //Read the incoming byte
        incomingByte = Serial.read();
        //Start the message when the '<' symbol is received
        if(incomingByte == '<') {
            started = true;
            index = 0;
        }
        //End the message when the '>' symbol is received
        else if(incomingByte == '>') {
            ended = true;
            break; // Done reading - exit from while loop
        }
        //Read the message!
        else {
            if(index < LENGTH) {  // Make sure there is roof
                msg[index] = incomingByte; // Add char to array
                cool_word += msg[index];
                index++;
            }
        }
    }

    if (started && ended) {
        Serial.print(cool_word);
        cool_word = "";
        index = 0;
        msg[index] = '\0';
        started = false;
        ended = false;
    }
}
