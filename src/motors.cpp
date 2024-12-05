#include <Arduino.h>
#include "motors.h"

Motors *MotorsSingleton::m = nullptr;

Motors::Motors()
{
    motor_fl.attach(MOTOR_FL, MIN_PW, MAX_PW);
    motor_fr.attach(MOTOR_FR, MIN_PW, MAX_PW);
    motor_rl.attach(MOTOR_RL, MIN_PW, MAX_PW);
    motor_rr.attach(MOTOR_RR, MIN_PW, MAX_PW);
}

void Motors::writeLeft(int intensity)
{
    motor_fl.write(intensity);
    motor_rl.write(intensity);
}

void Motors::writeRight(int intensity)
{
    motor_fr.write(intensity);
    motor_rr.write(intensity);
}

void Motors::writeFront(int intensity)
{
    motor_fl.write(intensity);
    motor_fr.write(intensity);
}

void Motors::writeRear(int intensity)
{
    motor_rl.write(intensity);
    motor_rr.write(intensity);
}

void Motors::writeFL(int intensity)
{
    if (intensity < 0 || intensity > 180)
    {
        intensity = 0;
    }
    motor_fl.write(intensity);
}

void Motors::writeFR(int intensity)
{
    if (intensity < 0 || intensity > 180)
    {
        intensity = 0;
    }
    motor_fr.write(intensity);
}

void Motors::writeRL(int intensity)
{
    if (intensity < 0 || intensity > 180)
    {
        intensity = 0;
    }
    motor_rl.write(intensity);
}

void Motors::writeRR(int intensity)
{
    if (intensity < 0 || intensity > 180)
    {
        intensity = 0;
    }
    motor_rr.write(intensity);
}

void Motors::writeAll(int intensity)
{
    motor_fl.write(intensity);
    motor_fr.write(intensity);
    motor_rl.write(intensity);
    motor_rr.write(intensity);
}

void Motors::writeDronePosition(DronePosition p)
{
    float frontIntensity = p.pitch;
    float rearIntensity = -p.pitch;
    float rightIntensity = -p.roll;
    float leftIntensity = p.roll;

    // float motorFl = p.throttle + leftIntensity + frontIntensity;
    // float motorFR = p.throttle + rightIntensity + frontIntensity;
    // float motorRL = p.throttle + leftIntensity + rearIntensity;
    // float motorRR = p.throttle + rightIntensity + rearIntensity;

    // Serial.println(String(motorFl) + " " + String(motorFR) + " " + String(motorRL) + " " + String(motorRR));

    writeFL(p.throttle + leftIntensity + frontIntensity + p.yaw);
    writeFR(p.throttle + rightIntensity + frontIntensity - p.yaw);
    writeRL(p.throttle + leftIntensity + rearIntensity - p.yaw);
    writeRR(p.throttle + rightIntensity + rearIntensity + p.yaw);
}

void Motors::testMotors()
{
    writeFL(70);
    delay(1000);
    writeFR(70);
    delay(1000);
    writeRL(70);
    delay(1000);
    writeRR(70);
    delay(99999999);
}