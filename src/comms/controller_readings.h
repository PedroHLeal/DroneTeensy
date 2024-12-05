#ifndef CONTROLLER_READINGS
#define CONTROLLER_READINGS

typedef struct
{
    float targetThrottle = 0, setPointPitch = 0, setPointRoll = 0;
    float desiredHeight = 0;
    bool armed = false;
} ControllerReadings;

#endif