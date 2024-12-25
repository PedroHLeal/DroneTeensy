#ifndef FILTERS_H
#define FILTERS_H

#include "lib/BasicLinearAlgebra/BasicLinearAlgebra.h"

void kalman1d(
    float KalmanState,
    float KalmanUncertainty,
    float KalmanInput,
    float KalmanInputAccuracy,
    float KalmanMeasurement,
    float KalmanMeasurementAccuracy,
    float *output,
    float *uncertainty,
    float dt);

class Kalman2d {
private:
    float InputAccuracy = 0, MeasurementAccuracy = 0;
    BLA::Matrix<2, 2, float> F;
    BLA::Matrix<2, 1, float> G;
    BLA::Matrix<2, 2, float> P;
    BLA::Matrix<2, 2, float> Q;
    BLA::Matrix<2, 1, float> S;
    BLA::Matrix<1, 2, float> H;
    BLA::Matrix<2, 2, float> I;
    BLA::Matrix<2, 1, float> K;
    BLA::Matrix<1, 1, float> R;
    BLA::Matrix<1, 1, float> L;
    BLA::Matrix<1, 1, float> M;
    BLA::Matrix<1, 1, float> Input;

public:
    float s00 = 0;
    float s10 = 0;
    Kalman2d(float InputAccuracy, float MeasurementAccuracy);
    float filter(
        float Input,
        float Measurement,
        float dt
    );
};

#endif