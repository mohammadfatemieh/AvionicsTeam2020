#ifndef XBEE_RECIEVER_H
#define XBEE_RECIEVER_H

void xbee_Serial_init();

// Returns a the first letter recieved from a message, otherwise '\0' if nothing recieved

char xbee_recieve();

// Sends word through xbee
void xbee_transmit(String cool_word);

// Check if recieved a letter
int xbee_Serial_available();

// Sends number through xbee
void xbee_transmit(int value);

#endif /* XBEE_RECIEVER_H */
