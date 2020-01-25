/*
 * Arduino.h
 *
 *  Created on: Jan 20, 2020
 *      Author: jonas
 */

#ifndef INC_ARDUINO_H_
#define INC_ARDUINO_H_

#include <stm32f1xx_hal.h>

inline uint32_t millis (void)
{
   return HAL_GetTick();
}

#define min(a,b) \
  ({ __typeof__ (a) _a = (a); \
      __typeof__ (b) _b = (b); \
    _a < _b ? _a : _b; })

#define max(a,b) \
  ({ __typeof__ (a) _a = (a); \
      __typeof__ (b) _b = (b); \
    _a > _b ? _a : _b; })


//#define min(a,b) (((a)<(b))?(a):(b))
//#define max(a,b) (((a)>(b))?(a):(b))

#endif /* INC_ARDUINO_H_ */
