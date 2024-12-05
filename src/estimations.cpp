#include "estimations.h"
#include "filters.h"

void Estimator::calculateEstimations(Gyro *g, PX4Flow *px4flow, RangeSensor *rs, float dt)
{
    float px4ReadX = -px4flow->get_vel_y(rs->distance);
    float px4ReadY = -px4flow->get_vel_x(rs->distance);

    if (isnan(px4ReadX) || isinf(px4ReadX)) {
        px4ReadX = 0;
    }
    if (isnan(px4ReadY) || isinf(px4ReadY)) {
        px4ReadY = 0;
    }
    // Serial.println(px4Read);

    kalman1d(estimatedVelX, velXUncertainty, g->inertialAccelX * 9.8 * 8, px4ReadX, &estimatedVelX, &velXUncertainty, dt);
    // estimatedVelY += g->inertialAccelY * 9.81 * 15 * dt;
    kalman1d(estimatedVelY, velYUncertainty, g->inertialAccelY * 9.8 * 8, px4ReadY, &estimatedVelY, &velYUncertainty, dt);
    kalman1d(estimatedVelZ, velZUncertainty, (g->inertialAccelZ - 1) * 9.81 * 8, rs->distance - rs->previousDistance, &estimatedVelZ, &velZUncertainty, dt);

    // Serial.println(String(estimatedVelZ));

    lastPX4ReadX = px4ReadX;
    lastPX4ReadY = px4ReadY;
}