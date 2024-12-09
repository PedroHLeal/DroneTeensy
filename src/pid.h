#ifndef PID_H
#define PID_H

#include "comms/bluetooth.h"
#include "comms/controller_readings.h"
#include "sensors/range_sensor.h"
#include "sensors/gyro.h"
#include "sensors/px4flow.h"
#include "estimations.h"

#define THROTTLE_P_GAIN 0.1
#define THROTTLE_I_GAIN 0.001
#define THROTTLE_D_GAIN 4

#define VEL_P_GAIN 1
#define VEL_I_GAIN 0.0
#define VEL_D_GAIN 2

#define PITCH_ROLL_P_GAIN 8

#define PITCH_ROLL_RATE_P_GAIN 0.03
#define PITCH_ROLL_RATE_I_GAIN 0.0008
#define PITCH_ROLL_RATE_D_GAIN 0.3

#define YAW_P_GAIN 0.5
#define MAX_RATE 20

typedef struct
{
    float armed = 0;

    float velX = 0;
    float velXI = 0;
    float previousVelXError = 0;
    float usePositioning = false;

    float velY = 0;
    float velYI = 0;
    float previousVelYError = 0;

    float setPointPitch = 0;
    float setPointRoll = 0;

    float throttle = 0;
    float throttleI = 0;
    float previousThrottleError = 0;

    float pitch = 0;
    float pitchI = 0;
    float previousPitchError = 0;

    float roll = 0;
    float rollI = 0;
    float previousRollError = 0;

    float pitchRate = 0;
    float pitchRateI = 0;
    float previousPitchRateError = 0;

    float rollRate = 0;
    float rollRateI = 0;
    float previousRollRateError = 0;

    float yaw = 0;
    bool emergencyQuit = false;
} DronePosition;

void calculatePidThrottle(DronePosition *p, ControllerReadings r, Estimator* e);
void calculatePidVelX(DronePosition *p, ControllerReadings r, Estimator* e);
void calculatePidVelY(DronePosition *p, ControllerReadings r, Estimator* e);
void calculatePidPitch(DronePosition *p, ControllerReadings r, Gyro* g);
void calculatePidRoll(DronePosition *p, ControllerReadings r, Gyro* g);
void calculatePidPitchRate(DronePosition *p, ControllerReadings r, Gyro* g);
void calculatePidRollRate(DronePosition *p, ControllerReadings r, Gyro* g);
void calculatePidYaw(DronePosition *p, Gyro* g);
void resetPid(DronePosition* p);

#endif