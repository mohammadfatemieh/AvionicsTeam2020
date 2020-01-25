#pragma once

class PID_Controller {
    private:
        double Kp;
        double Ki;
        double Kd;
        double dt;
        double maxInput;
        double minInput;
        double prevError;
        double integralError;
    public:
        PID_Controller(double Kp, double Ki, double Kd, double dt, double maxInput, double minVal);
        double calculateInput(double reference, double current_pos);
};