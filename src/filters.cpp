#include "filters.h"

void kalman1d(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement, float *output, float *uncertainty, float dt)
{
    KalmanState = KalmanState + dt * KalmanInput;
    KalmanUncertainty = KalmanUncertainty + dt * 4 * 4;
    float KalmanGain = KalmanUncertainty * 1 / (1 * KalmanUncertainty + 3 * 3);
    KalmanState = KalmanState + KalmanGain * (KalmanMeasurement - KalmanState);
    KalmanUncertainty = (1 - KalmanGain) * KalmanUncertainty;
    *output = KalmanState;
    *uncertainty = KalmanUncertainty;
}