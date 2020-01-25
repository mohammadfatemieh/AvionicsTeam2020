#pragma once

#define COUNTS_REVOLUTION 6678.624
#define CYCLES_REVOLUTION 1669.656

//Converts encoder counts to degree, use interrupts??
double countsToDegree(int counts);

//Converts encoder counts to radians
double countsToRad(int counts);

// TODO: Find out how to read encoder counts on stm32