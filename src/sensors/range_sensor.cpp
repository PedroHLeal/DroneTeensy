#include <Wire.h>
#include "range_sensor.h"

RangeSensor *RangeSensorSingleton::sensors[2] = {new GY_US42(), new SR04()};

void GY_US42::setup()
{
}

void GY_US42::update()
{
    Wire.beginTransmission(address);
    Wire.write(command);
    Wire.endTransmission();
    delay(10);
    Wire.requestFrom(address, byte(2));
    if (Wire.available())
    {
        byte HighByte = Wire.read();
        byte LowByte = Wire.read();
        word range = word(HighByte, LowByte);

        previousDistance = distance;
        distance = range;
    }
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
    filter = new FilterOnePole(LOWPASS, 5);
}

void SR04::setup()
{
}

void SR04::update()
{
    previousDistance = distance;

    digitalWrite(trigger, LOW);
    delayMicroseconds(2);
    digitalWrite(trigger, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigger, LOW);
    distance = (pulseIn(echo, HIGH) * 0.343) / 2 - 47;
    filter->input(distance);
    distance = filter->output();
}

void SR04::test()
{
    this->update();
    Serial.println(distance);
}