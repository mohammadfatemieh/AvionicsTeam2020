/*
 * USART.h
 *
 * Created: 05/09/2019 18.50.19
 *  Author: MatiusH
 */ 

#ifndef USART_H_
#define USART_H_

#include <avr/io.h>

#define FOSC 16000000
#define BAUD 19200
#define MYUBRR FOSC/16/BAUD-1
#define RX_BUFFER_LEN 64

extern char NEWLINE[2];
extern char READY[7];

void USART_init(unsigned int ubrr);

unsigned char USART_receive(void);

void USART_transmit(char *buffer, uint32_t lenght);

void USART_responder(void);

#endif
