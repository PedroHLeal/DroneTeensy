#include "estimations.h"

void Estimator::calculateEstimations(Gyro *g, MTF02P* mtf02p, float dt)
{
    MICOLINK_PAYLOAD_RANGE_SENSOR_t optFlow = mtf02p->payload;
    k2d->filter((g->inertialAccelZ - 1) * 9.81 * 1000, optFlow.distance, dt);
    positionZ = k2d->s00;
    velZ = k2d->s10;
    isPositioningAvailable = true;
    if (optFlow.flow_quality < 70 || positionZ < 200)
    {
        isPositioningAvailable = false;
        estimatedVelX = 0;
        estimatedVelY = 0;
        estimatedPosX = 0;
        estimatedPosY = 0;
        return;
    }

    float flowX = (-optFlow.flow_vel_y * 0.5) + g->gY;
    float flowY = (-optFlow.flow_vel_x * 0.5) - g->gX;

    filterFLowX->input(flowX);
    flowX = filterFLowX->output();
    filterFLowY->input(flowY);
    flowY = filterFLowY->output();

    flowX = (flowX * positionZ)/1000;
    flowY = (flowY * positionZ)/1000;

    kalman1d(estimatedVelX, velXUncertainty, g->inertialAccelX * 9.81, 10, flowX, 3, &estimatedVelX, &velXUncertainty, dt);
    kalman1d(estimatedVelY, velYUncertainty, g->inertialAccelY * 9.81, 10, flowY, 3, &estimatedVelY, &velYUncertainty, dt);

    // estimatedVelX += g->inertialAccelX * 9.8;
    // estimatedVelY = flowY * 98;

    estimatedPosX += estimatedVelX * dt;
    estimatedPosY += estimatedVelY * dt;

    // Serial.println(String(estimatedVelY) + " " + String(flowY));
}

void Estimator::resetEstimations()
{
    estimatedVelX = 0;
    estimatedVelY = 0;
    estimatedVelZ = 0;
    estimatedPosX = 0;
    estimatedPosY = 0;
    positionZ = 0;
    velZ = 0;
}
