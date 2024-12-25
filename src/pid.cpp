#include "pid.h"
#include "sensors/range_sensor.h"
#include "lib/DSPFilters/src/Filters.h"
#include "sensors/gyro.h"

FilterOnePole *lpFilter = new FilterOnePole(LOWPASS, 5);

void calculatePidPosZ(DronePosition *p, ControllerReadings r, Estimator *e)
{
    p->targetVelZ = (r.targetThrottle - e->positionZ/10) * VEL_Z_P;
    // Serial.println(p->targetVelZ);
}

void calculatePidThrottle(DronePosition *p, ControllerReadings r, Estimator *e)
{
    float targetVel = p->useZPositioning ? p->targetVelZ : r.targetThrottle * 10;
    float throttleError = targetVel - (e->velZ / 10 - 6);
    p->throttleI += throttleError * THROTTLE_I_GAIN;

    p->throttle = constrain(throttleError * THROTTLE_P_GAIN + p->throttleI + (throttleError - p->previousThrottleError) * THROTTLE_D_GAIN, 0, 140);
    // p->throttle = r.targetThrottle;
    // Serial.println(p->throttle);
    p->previousThrottleError = throttleError;
}

void calculatePidPosX(DronePosition *p, ControllerReadings r, Estimator *e)
{
    p->targetVelX = (r.setPointRoll - e->estimatedPosX) * POS_P_GAIN;
    // Serial.println(p->targetVelX);
}

void calculatePidPosY(DronePosition *p, ControllerReadings r, Estimator *e)
{
    p->targetVelY = (-r.setPointPitch - e->estimatedPosY) * POS_P_GAIN;
    // Serial.println(p->targetVelY);
}

void calculatePidVelX(DronePosition *p, ControllerReadings r, Estimator *e)
{
    float setPointVelX = p->targetVelX;
    if (!p->usePositioning)
    {
        setPointVelX = r.setPointRoll*2;
    }
    // Serial.println(p->targetVelX);
    float velXError = setPointVelX - e->estimatedVelX;
    p->velXI += constrain(velXError * VEL_I_GAIN, -5, 5);
    p->setPointRoll = constrain(velXError * VEL_P_GAIN + p->velXI + (velXError - p->previousVelXError) * VEL_D_GAIN, -10, 10);
    // Serial.println(p->setPointRoll);
    p->previousVelXError = velXError;
}

void calculatePidVelY(DronePosition *p, ControllerReadings r, Estimator *e)
{
    float setPointVelY = p->targetVelY;
    if (!p->usePositioning)
    {
        setPointVelY = -r.setPointPitch*2;
    }

    float velYError = setPointVelY - e->estimatedVelY;
    p->velYI += constrain(velYError * VEL_I_GAIN, -5, 5);
    p->setPointPitch = -constrain(velYError * VEL_P_GAIN + p->velYI + (velYError - p->previousVelYError) * VEL_D_GAIN, -10, 10);
    // Serial.println(p->setPointPitch);
    p->previousVelYError = velYError;
}

void calculatePidPitch(DronePosition *p, ControllerReadings r, Gyro *g)
{
    float spp = r.setPointPitch;
    if (p->usePositioning)
    {
        spp = p->setPointPitch;
    }
    p->pitchRate = (spp - g->gPosX) * PITCH_ROLL_P_GAIN;
    // Serial.println(p->pitchRate);
}

void calculatePidRoll(DronePosition *p, ControllerReadings r, Gyro *g)
{
    float spr = r.setPointRoll;
    if (p->usePositioning)
    {
        spr = p->setPointRoll;
    }
    p->rollRate = (spr - g->gPosY) * PITCH_ROLL_P_GAIN;
}

void calculatePidPitchRate(DronePosition *p, ControllerReadings r, Gyro *g)
{
    float pitchRateError = p->pitchRate - g->gX;
    p->pitchRateI += constrain(pitchRateError * PITCH_ROLL_RATE_I_GAIN, -5, 5);
    p->pitch = pitchRateError * PITCH_ROLL_RATE_P_GAIN + p->pitchRateI + (pitchRateError - p->previousPitchRateError) * PITCH_ROLL_RATE_D_GAIN;
    p->pitch = constrain(p->pitch, -MAX_RATE, MAX_RATE);
    p->previousPitchRateError = pitchRateError;
}

void calculatePidRollRate(DronePosition *p, ControllerReadings r, Gyro *g)
{
    float rollRateError = p->rollRate - g->gY;
    p->rollRateI += constrain(rollRateError * PITCH_ROLL_RATE_I_GAIN, -5, 5);
    p->roll = rollRateError * PITCH_ROLL_RATE_P_GAIN + p->rollRateI + (rollRateError - p->previousRollRateError) * PITCH_ROLL_RATE_D_GAIN;
    p->roll = constrain(p->roll, -MAX_RATE, MAX_RATE);
    p->previousRollRateError = rollRateError;
}

void calculatePidYaw(DronePosition *p, Gyro *g)
{
    p->yaw = g->gPosZ * YAW_P_GAIN;
}

void resetPid(DronePosition *p)
{
    p->armed = 0;
    p->throttle = 0;
    p->throttleI = 0;
    p->previousThrottleError = 0;
    p->pitch = 0;
    p->pitchI = 0;
    p->previousPitchError = 0;
    p->roll = 0;
    p->rollI = 0;
    p->previousRollError = 0;
    p->pitchRate = 0;
    p->pitchRateI = 0;
    p->previousPitchRateError = 0;
    p->rollRate = 0;
    p->rollRateI = 0;
    p->previousRollRateError = 0;
    p->yaw = 0;
    p->emergencyQuit = 0;
    p->velXI = 0;
    p->velYI = 0;
}