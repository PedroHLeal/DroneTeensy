#include "filters.h"

void kalman1d(
    float KalmanState,
    float KalmanUncertainty,
    float KalmanInput,
    float KalmanInputAccuracy,
    float KalmanMeasurement,
    float KalmanMeasurementAccuracy,
    float *output,
    float *uncertainty,
    float dt)
{
    KalmanState = KalmanState + dt * KalmanInput;
    KalmanUncertainty = KalmanUncertainty + dt * KalmanInputAccuracy * KalmanInputAccuracy;
    float KalmanGain = KalmanUncertainty * 1 / (1 * KalmanUncertainty + KalmanMeasurementAccuracy * KalmanMeasurementAccuracy);
    KalmanState = KalmanState + KalmanGain * (KalmanMeasurement - KalmanState);
    KalmanUncertainty = (1 - KalmanGain) * KalmanUncertainty;
    *output = KalmanState;
    *uncertainty = KalmanUncertainty;
}

Kalman2d::Kalman2d(float InputAccuracy, float MeasurementAccuracy)
{
    this->InputAccuracy = InputAccuracy;
    this->MeasurementAccuracy = MeasurementAccuracy;
    F = {1, 0.004,
         0, 1};
    G = {0.5 * 0.004 * 0.004,
         0.004};
    H = {1, 0};
    I = {1, 0,
         0, 1};
    Q = G * ~G * InputAccuracy * InputAccuracy;
    R = {MeasurementAccuracy * MeasurementAccuracy};
    P = {0, 0,
         0, 0};
    S = {0,
         0};
}

float Kalman2d::filter(
    float Input,
    float Measurement,
    float dt)
{
    F = {1, dt,
         0, 1};
    G = {0.5f * dt * dt,
         dt};
    Q = G * ~G*InputAccuracy*InputAccuracy;

    this->Input = {Input};
    S = F * S + G * this->Input;
    P = F * P * ~F + Q;
    L = H * P * ~H + R;
    K = P * ~H * Inverse(L);
    M = {Measurement};
    S = S + K * (M - H * S);
    P = (I - K * H) * P;
    return S(0, 0);
}
