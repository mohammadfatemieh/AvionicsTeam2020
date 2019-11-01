/*
 * USART.c
 *
 * Created: 05/09/2019 18.47.44
 *  Author: MatiusH
 */ 

#include <avr/io.h>
#include "USART.h"

char NEWLINE[2] = {'\n', '\r'};
char READY[7] = {'R', 'e', 'a', 'd', 'y', '\n', '\r'};


void USART_init(unsigned int ubrr)
{
	// Set baud rate
	UBRR0H = (unsigned char) (ubrr>>8);
	UBRR0L = (unsigned char) ubrr;
	// Enable RX and TX
	UCSR0B = (1 << RXEN0) | (1 << TXEN0);
	// Asynchronous 8-bit data, no parity, 1 stop bit
	UCSR0C |= (1 << UCSZ01) | (1 << UCSZ00);
}

unsigned char USART_receive(void)
{
	while ( !(UCSR0A & (1<<RXC0)) );	// Wait while receiving data
	return UDR0;
}
void USART_transmit(char *buffer, uint32_t lenght)
{
	for (uint32_t i = 0; i < lenght; i++)
	{
		// Wait until transmit buffer is empty
		while (!(UCSR0A & (1 << UDRE0)));
		// Transmit char
		UDR0 = buffer[i];
	}
}


void USART_responder(void)
{
	// Setup variables
	unsigned char newChar;
	char rx_buffer[RX_BUFFER_LEN];
	uint8_t rx_index = 0;
	
	while(1)
	{
		newChar = USART_receive();
		if (newChar != '\n' && newChar != '\r')
		{
			// Received a character that was not \n or \r
			// Add character to rx_buffer
			rx_buffer[rx_index++] = newChar;
			// Check for buffer overflow
			if (rx_index == RX_BUFFER_LEN)
			{
				rx_index = 0;
			}
		} else if ((rx_buffer[0] != '\n' || rx_buffer[0] != '\r') && rx_index > 0)
		{
			// Received \n or \r and buffer was not empty
			// Transmit the contents of rx_buffer
			USART_transmit(&rx_buffer, rx_index);
			USART_transmit(&NEWLINE, (uint8_t) 2);
			// Zero rx_index to be ready for a new transmission
			rx_index = 0;
		}
	}
}
