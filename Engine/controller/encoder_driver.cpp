#include "encoder_driver.h"
#define _USE_MATH_DEFINES
#include <math.h>


double countsToDegree(int counts) {
    return  360 * counts/COUNTS_REVOLUTION;
}

double countsToRad(int counts) {
    return  2*M_PI * counts/COUNTS_REVOLUTION;
}