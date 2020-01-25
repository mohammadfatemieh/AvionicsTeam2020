#include "pid.h"
#include <cmath>

PID_Controller::PID_Controller(double Kp, double Ki, double Kd, double dt, 
        double maxInput, double minInput) : // initialiser list
    Kp(Kp), 
    Ki(Ki), 
    Kd(Kd), 
    dt(dt), 
    maxInput(maxInput), 
    minInput(minInput), 
    prevError(0), 
    integralError(0)
{
}

double PID_Controller::calculateInput(double reference, double current_pos) {
    double error = reference - current_pos;
    integralError += error;

    double proportional_term = Kp * error;
    double integral_term = Ki * integralError;
    double derivative_term = Kd * (error - prevError) / dt;

    double inputSignal = proportional_term + integral_term + derivative_term;

    prevError = error;

    if (inputSignal > maxInput) {
        return maxInput;
    } else if (inputSignal < minInput) {
        return minInput;
    }
    return inputSignal;
}
