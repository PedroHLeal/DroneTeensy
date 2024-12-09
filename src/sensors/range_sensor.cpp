#include <Wire.h>
#include "range_sensor.h"

RangeSensor *RangeSensorSingleton::sensors[2] = {new GY_US42(), new SR04()};

void GY_US42::setup()
{
}

GY_US42::GY_US42()
{
}

void GY_US42::update()
{
    Wire.beginTransmission(address);
    Wire.write(command);
    Wire.endTransmission();
    Wire.requestFrom(address, byte(2));
    // delay(2);
    // Serial.println("GY");
    while (!(Wire.available() >= 2))
        ;
    byte HighByte = Wire.read();
    byte LowByte = Wire.read();
    word range = word(HighByte, LowByte);
    previousDistance = distance;
    if ((range*10 - distance) < 50 && (range*10 - distance) > -50 )
    {
        // Serial.println(range*10);
        distance = range * 10;
    }
    // filter->input(range * 10);
    // distance = filter->output();
}

void GY_US42::test()
{
    this->update();
    Serial.println(distance);
}

// ------------------------------------------------------------------
// ------------------------------------------------------------------
// ------------------------------------------------------------------

SR04::SR04()
{
    pinMode(trigger, OUTPUT);
    pinMode(echo, INPUT);
    sonar = new NewPing(trigger, echo, 200);
}

void SR04::setup()
{
}

void SR04::update()
{
    // Serial.println("sr04");
    float range = sonar->ping_cm() * 10;
    // Serial.println(range);
    if (range < 3000 && range > 0 )
    {
        // Serial.println(range*10);
        previousDistance = distance;
        distance = range;
    }
    // filter->input(distance);
    // distance = filter->output();
}

void SR04::test()
{
    update();
}