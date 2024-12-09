#ifndef _PROX_SENSOR
#define _PROX_SENSOR

#include "lib/NewPing/src/NewPing.h"

class RangeSensor
{
public:
    float previousDistance = 0.0;
    float distance = 0.0;
    float velocity = 0.0;

    virtual void update() = 0;
    virtual void setup() = 0;
    virtual void test() = 0;
};

class GY_US42 : public RangeSensor
{
public:
    GY_US42();
    void update();
    void setup();
    void test();

private:
    int address = byte(0x70);
    int command = byte(0x51);
};


class SR04 : public RangeSensor
{
public:
    SR04();
    void update();
    void setup();
    void test();

private:
    NewPing *sonar;
    int trigger = 9, echo = 6;
};

#define GYUS42 0
#define HC_SR04 1

class RangeSensorSingleton
{
public:
    static RangeSensor *getInstance(int sensor)
    {
        return sensors[sensor];
    }

private:
    static RangeSensor *sensors[2];
};

#endif