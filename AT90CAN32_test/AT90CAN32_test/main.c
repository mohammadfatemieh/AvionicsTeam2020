/*
 * AT90CAN32_test.c
 *
 * Created: 22/10/2019 11.10.42
 * Author : MatiusH
 */ 

#define F_CPU 16000000UL

#include <avr/io.h>
#include <util/delay.h>
#include "USART.h"


int main(void)
{
	// Set led as output
	DDRB |= (1 << DDB1);
	// Set led on
	PORTB |= (1 << PB1);
	// Init USART
	USART_init(MYUBRR);
	
    // Blink the LED
    while (1) 
    {
		PORTB ^= (1 << PB1);
		_delay_ms(500);
    }
	
	// Init I2C
	I2C_init();
	// Create a buffer for MPU-6050 readings
	uint8_t MPU6050_result[12];
	
	while (1)
	{
		// Read accelerometer and gyroscope readings
		MPU6050_read_all(&MPU6050_result);
		
		// Convert results to chars and transmit them via USART
		// Create a buffer for the conversion
		char buffer[6];
		for (uint8_t i=0; i<6; i++)
		{
			// Extract the 16-bit value from the 8-bit registers
			uint16_t val = MPU6050_result[2*i+1];
			// Convert
			itoa(val, buffer, 10);
			// Transmit result
			USART_transmit(&buffer, 6);
			USART_transmit(' ', 1);
			// Empty char buffer
			for (uint8_t j=0; i<6; i++)
			{
				buffer[j] = ' ';
			}
		}
		// Transmit newline
		USART_transmit(NEWLINE, 2);
		
		// Sleep
		_delay_ms(1000);
	}
}

main();
