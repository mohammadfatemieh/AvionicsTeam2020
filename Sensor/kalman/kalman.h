#include "BasicLinearAlgebra/BasicLinearAlgebra.h"
using namespace BLA;

class Kalman {
private:                    //dimensions below are temporary, need to be double-checked. initial values must also be found.
    Matrix<2,2> A;
    Matrix<2,1> B;
    const Matrix<1,2> C = ???;
    const float R = ???;            //covariance (uncertainty) in measurement; tuning variable.
    const Matrix<2,2> Q = ???;      //covariance in disturbance; tuning var.
    const Matrix<2,2> I = {1, 0, 0, 1};

    Matrix<2,1> K;      //Kalman gain
    Matrix<2,2> P_hat;  //a posteriori
    Matrix<2,2> P_bar;  //a priori
    Matrix<2,1> x_hat;
    Matrix<2,1> x_bar;  //note that x_bar = x_hat_bar = a priori

    
public:
    Kalman(Matrix<2,2> A1, Matrix<2,1> B1, Matrix<2,2> P1, Matrix<2,1> x1){A = A1; B = B1; P_bar = P1; x_bar = x1;}

    //Correcting with new data
    void updateKalmanGain();
    void updateXHat(float measurement);
    void updatePMatrix(float measurement);

    //Predicting ahead
    void updateXBar(float input);
    void updatePBar();
 
    Matrix<2,1> getStates();
};