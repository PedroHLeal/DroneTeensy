#ifndef ESTIMATIONS_H
#define ESTIMATIONS_H

#include "sensors/gyro.h"
#include "sensors/px4flow.h"
#include "sensors/range_sensor.h"

class Estimator {
public:
    float estimatedVelX = 0, estimatedVelY = 0, estimatedVelZ;
    float velXUncertainty = 0, velYUncertainty = 0, velZUncertainty = 0;
    void calculateEstimations(Gyro* g, PX4Flow* px4flow, RangeSensor* rs, float dt);
private:
    float lastPX4ReadX = 0, lastPX4ReadY = 0;
};

#endif