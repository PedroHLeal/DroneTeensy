#ifndef ESTIMATIONS_H
#define ESTIMATIONS_H

#include "sensors/gyro.h"
#include "sensors/mtf02p.h"
#include "filters.h"
#include "lib/DSPFilters/src/Filters.h"

class Estimator {
public:
    float estimatedVelX = 0, estimatedVelY = 0, estimatedVelZ;
    float estimatedPosX = 0, estimatedPosY = 0, estimatedPosZ;
    float velXUncertainty = 0, velYUncertainty = 0, velZUncertainty = 0;
    float positionZ = 0, velZ = 0;
    bool isPositioningAvailable = false;
    void calculateEstimations(Gyro* g, MTF02P* mtf02p, float dt);
    void resetEstimations();
private:
    FilterOnePole *lowPass = new FilterOnePole(LOWPASS, 5);
    FilterOnePole *filterFLowX = new FilterOnePole(LOWPASS, 5);
    FilterOnePole *filterFLowY = new FilterOnePole(LOWPASS, 5);
    Kalman2d *k2d = new Kalman2d(50, 0.05);
};

#endif