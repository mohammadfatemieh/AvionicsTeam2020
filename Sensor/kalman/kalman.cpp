#include "kalman.h"
#include "BasicLinearAlgebra/BasicLinearAlgebra.h"
using namespace BLA;

void Kalman::updateKalmanGain(){
    K = P_bar*(~C) * (( C*P_bar*(~C) + R ).Inverse());
}

void Kalman::updateXHat(float measurement){
    if (measurement){
        x_hat = x_bar + K*( measurement - C*x_bar );
    }else{
        x_hat = x_bar;  //if no new measurement available, predicted x = estimated x.
    }
}

void Kalman::updatePMatrix(float measurement){
    if (measurement){
        P_hat = (I - K*C)*P_bar*(~(I-K*C)) + K*R*(~K);
    }else{
        P_hat = P_bar;  //if no new measurement available, predicted P = estimated P.
    }
}

void Kalman::updateXBar(float input){
    x_bar = A*x_hat + B*input;
}

void Kalman::updatePBar(){
    P_bar = A*P_hat*(~A) + Q;
}

Matrix<2,1> Kalman::getStates(){
    return x_hat;
}