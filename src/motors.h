#ifndef _MOTORS
#define _MOTORS

#include <Servo.h>
#include "pid.h"

// SETTING MOTOR PINS
#define MOTOR_FL 5
#define MOTOR_FR 4
#define MOTOR_RL 3
#define MOTOR_RR 2

#define MIN_PW 1000
#define MAX_PW 2000

class Motors
{
private:
  Servo motor_fl, motor_fr, motor_rl, motor_rr;

public:
  Motors();
  void writeLeft(int intensity);
  void writeRight(int intensity);
  void writeFront(int intensity);
  void writeRear(int intensity);
  void writeFL(int intensity);
  void writeFR(int intensity);
  void writeRL(int intensity);
  void writeRR(int intensity);
  void writeAll(int intensity);
  void writeDronePosition(DronePosition p);
  void testMotors();
};

class MotorsSingleton
{
public:
  static Motors *m;
  static Motors *getInstance()
  {
    if (m == nullptr)
    {
      m = new Motors();
    }

    return m;
  }
};

#endif