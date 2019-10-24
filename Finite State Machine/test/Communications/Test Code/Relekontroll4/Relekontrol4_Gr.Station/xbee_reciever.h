#ifndef XBEE_RECIEVER_H
#define XBEE_RECIEVER_H


// Returns a the first letter recieved from a message, otherwise '\0' if nothing recieved

char xbee_recieve();

// Sends word through xbee
void xbee_transmit(String word);

#endif /* XBEE_RECIEVER_H */
