
/*
 * I2C.h
 *
 * Created: 31/10/2019 12.21.44
 *  Author: MatiusH
 */ 

#ifndef I2C_H_
#define I2C_H_

#include <avr/io.h>

extern char I2C_ERROR[];
char I2C_RECEIVE_BUF[64];

void I2C_init();

void I2C_transmit(uint8_t address, uint8_t *data);

void I2C_receive(uint8_t address, uint8_t *buffer, uint8_t len);

#endif