#ifndef ESTIMATIONS_H
#define ESTIMATIONS_H

#include "sensors/gyro.h"
#include "sensors/px4flow.h"
#include "sensors/range_sensor.h"
#include "filters.h"
#include "lib/DSPFilters/src/Filters.h"


class Estimator {
public:
    float estimatedVelX = 0, estimatedVelY = 0, estimatedVelZ;
    float velXUncertainty = 0, velYUncertainty = 0, velZUncertainty = 0;
    float positionY = 0;
    bool isPositioningAvailable = false;
    void calculateEstimations(Gyro* g, PX4Flow* px4flow, RangeSensor* rs, float dt);
private:
    FilterOnePole *lowPass = new FilterOnePole(LOWPASS, 5);
    void resetEstimations();
    float lastPX4ReadX = 0, lastPX4ReadY = 0;
    Kalman2d *k2d = new Kalman2d(100, 1);
    FilterOnePole* filter;
};

#endif