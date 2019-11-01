
/*
 * I2C.c
 *
 * Created: 31/10/2019 12.21.33
 *  Author: MatiusH
 */ 



#include "I2C.h"

char I2C_ERROR[] = {'E', 'R', 'R', 'O', 'R', '\n', '\r'};
	

void I2C_init()
{
	// Set bit rate to 200 kHz (provided TWPS in TWSR is 0)
	TWBR = 32;	// To get 400 kHz swap to 12
}


void I2C_transmit(uint8_t address, uint8_t *data)
{
	// Formulate SLA_W byte by left-shifting the 7-bit address,
	// generating a 0 at LSB (write operation)
	uint8_t SLA_W = (uint8_t) (address << 1);
	
	// Send start condition
	TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
	// Wait for TWINT flag set (indicating the start condition has been transmitted)
	while (!(TWCR & (1 << TWINT)));
	// Check value of TWI status register (masking prescaler bits)
	// If status differs from START print ERROR
	if ((TWSR & 0xf8) != 0x08)
	{
		USART_transmit(&I2C_ERROR, 7);
	}
	// Load SLA_W into TWDR
	TWDR = SLA_W;
	// Clear TWINT bit TWCR to start transmission of address
	TWCR = (1 << TWINT) | (1 << TWEN);
	// Wait for TWINT flag set (indicating SLA_W has been transmitted and ACK received)
	while (!(TWCR & (1 << TWINT)));
	// Check value of TWI status register (masking prescaler bits)
	// If status differs from MT_SLA_ACK print ERROR
	if ((TWSR & 0xf8) != 0x18)
	{
		USART_transmit(&I2C_ERROR, 7);
	}
	// Load data into TWDR
	TWDR = *data;
	// Clear TWINT bit TWCR to start transmission of data
	TWCR = (1 << TWINT) | (1 << TWEN);
	// Wait for TWINT flag set (indicating data has been transmitted and (N)ACK received)
	while (!(TWCR & (1 << TWINT)));
	// Check value of TWI status register (masking prescaler bits)
	// If status differs from MT_DATA_ACK print ERROR
	if ((TWSR & 0xf8) != 0x28)
	{
		USART_transmit(&I2C_ERROR, 7);
	}
	// Transmit stop condition
	TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);
}


void I2C_receive(uint8_t address, uint8_t *buffer, uint8_t len)
{
	// Formulate SLA_R byte by left-shifting the 7-bit address
	// and generate a 1 at LSB (read operation)
	uint8_t SLA_R = (uint8_t) (address << 1);
	SLA_R = (SLA_R & 0b11111110) | 0b00000001;
	
	// Send start condition
	TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
	// Wait for TWINT flag set (indicating the start condition has been transmitted)
	while (!(TWCR & (1 << TWINT)));
	// Check value of TWI status register (masking prescaler bits)
	// If status differs from START print ERROR
	if ((TWSR & 0xf8) != 0x08)
	{
		USART_transmit(&I2C_ERROR, 7);
	}
	// Load SLA_R into TWDR
	TWDR = SLA_R;
	// Clear TWINT bit TWCR to start transmission of address
	TWCR = (1 << TWINT) | (1 << TWEN);
	// Wait for TWINT flag set (indicating SLA_R has been transmitted and ACK received)
	while (!(TWCR & (1 << TWINT)));
	// Check value of TWI status register (masking prescaler bits)
	// If status differs from MR_SLA_ACK print ERROR
	if ((TWSR & 0xf8) != 0x40)
	{
		USART_transmit(&I2C_ERROR, 7);
	}
	// Read all but last byte of data from TWDR into buffer
	for (uint8_t i=0; i<(len-1); i++)
	{
		buffer[i] = TWDR;
		// Clear TWINT bit TWCR to enable reception of data
		TWCR = (1 << TWINT) | (1 << TWEN);
		// Wait for TWINT flag set (indicating data has been received)
		while (!(TWCR & (1 << TWINT)));
		// Check value of TWI status register (masking prescaler bits)
		// If status differs from MR_DATA_ACK print ERROR
		if ((TWSR & 0xf8) != 0x50)
		{
			USART_transmit(&I2C_ERROR, 7);
		}
	}
	// Read last byte of data from TWDR into buffer
	buffer[(len-1)] = TWDR;
	// Clear TWINT bit TWCR to do not enable further reception of data
	TWCR = (1 << TWINT);
	// Wait for TWINT flag set (indicating data has been received)
	while (!(TWCR & (1 << TWINT)));
	// Check value of TWI status register (masking prescaler bits)
	// If status differs from MR_DATA_NACK print ERROR
	if ((TWSR & 0xf8) != 0x58)
	{
		USART_transmit(&I2C_ERROR, 7);
	}
	// Transmit stop condition
	TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);
}