
/*
 * MPU6050.h
 *
 * Created: 31/10/2019 16.02.50
 *  Author: MatiusH
 */ 

#ifndef MPU6050_H_
#define MPU6050_H_

#define ACCEL_XOUT_H 0x3b
#define ACCEL_XOUT_L 0x3c
#define ACCEL_YOUT_H 0x3d
#define ACCEL_YOUT_L 0x3e
#define ACCEL_ZOUT_H 0x3f
#define ACCEL_ZOUT_L 0x40
#define GYRO_XOUT_H 0x43
#define GYRO_XOUT_L 0x44
#define GYRO_YOUT_H 0x45
#define GYRO_YOUT_L 0x46
#define GYRO_ZOUT_H 0x47
#define GYRO_ZOUT_L 0x48

#include <avr/io.h>
#include "I2C.h"

extern uint8_t MPU6050_ADDR;

void MPU6050_init();

void MPU6050_read_all(uint8_t *buffer);


#endif