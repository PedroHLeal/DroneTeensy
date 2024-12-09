#include "estimations.h"

void Estimator::calculateEstimations(Gyro *g, PX4Flow *px4flow, RangeSensor *rs, float dt)
{
    positionY = k2d->filter((g->inertialAccelZ - 1) * 9.81 * 1000, rs->distance, dt);
    float px4ReadX = -px4flow->get_vel_y(positionY);
    float px4ReadY = -px4flow->get_vel_x(positionY);
    isPositioningAvailable = true;
    if (px4flow->quality_integral() < 100 || isnan(px4ReadX) || isnan(px4ReadY) || isinf(px4ReadX) || isinf(px4ReadY))
    {
        isPositioningAvailable = false;
        px4ReadX = 0;
        px4ReadY = 0;
    }
    kalman1d(estimatedVelX, velXUncertainty, g->inertialAccelX * 9.8 * 8, 4, px4ReadX, 3, &estimatedVelX, &velXUncertainty, dt);
    kalman1d(estimatedVelY, velYUncertainty, g->inertialAccelY * 9.8 * 8, 4, px4ReadY, 3, &estimatedVelY, &velYUncertainty, dt);
    // Serial.println(String(estimatedVelX) + " " + String(estimatedVelY));

    lastPX4ReadX = px4ReadX;
    lastPX4ReadY = px4ReadY;
}

void Estimator::resetEstimations()
{
    estimatedVelX = 0;
    estimatedVelY = 0;
    estimatedVelZ = 0;
    positionY = 0;
}
