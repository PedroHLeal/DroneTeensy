#include "pid.h"
#include "sensors/range_sensor.h"
#include "lib/DSPFilters/src/Filters.h"
#include "sensors/gyro.h"

FilterOnePole *lpFilter = new FilterOnePole(LOWPASS, 5);

void calculatePidThrottle(DronePosition *p, ControllerReadings r, Estimator *e)
{
    float throttleError = r.targetThrottle - e->estimatedVelZ;
    p->throttleI += throttleError * THROTTLE_I_GAIN;

    lpFilter->input((throttleError - p->previousThrottleError) * THROTTLE_D_GAIN);
    p->throttle = throttleError * THROTTLE_P_GAIN + p->throttleI + (lpFilter->output());
    // p->throttle = r.targetThrottle;
    // Serial.println(p->throttle);
    p->previousThrottleError = throttleError;
}

void calculatePidVelX(DronePosition *p, ControllerReadings r, PX4Flow *px4, RangeSensor *rs)
{
    if (px4->quality_integral() < 100)
    {
        p->usePositioning = false;
        return;
    }
    p->usePositioning = true;
    float velXError = px4->get_vel_x(rs->distance) + r.setPointPitch;
    p->velXI += velXError * VEL_I_GAIN;
    p->setPointPitch = -(velXError * VEL_P_GAIN + p->velXI + (velXError - p->previousVelXError) * VEL_D_GAIN);
    p->previousVelXError = velXError;
}

void calculatePidVelY(DronePosition *p, ControllerReadings r, PX4Flow *px4, RangeSensor *rs)
{
    if (px4->quality_integral() < 100)
    {
        p->usePositioning = false;
        return;
    }
    p->usePositioning = true;
    float velYError = px4->get_vel_y(rs->distance) + r.setPointRoll;
    p->velYI += velYError * VEL_I_GAIN;
    p->setPointRoll = -(velYError * VEL_P_GAIN + p->velYI + (velYError - p->previousVelYError) * VEL_D_GAIN);
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
}