
/*
 * MPU6050.c
 *
 * Created: 31/10/2019 16.03.01
 *  Author: MatiusH
 */ 

#include "MPU6050.h"

void MPU6050_read_all(uint8_t *buffer)
{
	uint8_t acc_base = ACCEL_XOUT_H;
	uint8_t gyro_base = GYRO_XOUT_H;
	
	// Read accelerometer values
	for (uint8_t i=0; i<6; i++)
	{
		I2C_transmit(MPU6050_ADDR, (&acc_base)+i);
		I2C_receive(MPU6050_ADDR, &(buffer[i]), 1);
	}
	// Read gyroscope values
	for (uint8_t i=0; i<6; i++)
	{
		I2C_transmit(MPU6050_ADDR, (&gyro_base)+i);
		I2C_receive(MPU6050_ADDR, &(buffer[i+6]), 1);
	}
}