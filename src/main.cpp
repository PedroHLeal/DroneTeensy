#include <Arduino.h>
#include <Wire.h>
#include "motors.h"
#include "sensors/gyro.h"
#include "sensors/range_sensor.h"
#include "comms/controller_readings.h"
#include "comms/bluetooth.h"
#include "sensors/px4flow.h"
#include "sensors/mtf02p.h"
#include "pid.h"
#include "estimations.h"

// RUN_DRONE 0
// TEST_GYRO 2
// TEST_SONAR 3
// TEST_PX4FLOW 4
// CALIBRATE 5
// CALIBRATE MOTORS 6
// TEST MOTORS 7
// SHOW CALIBRATION VALUES 8
// TEST PX4FLOW 9
// CALIBRATE OPT_FLOW 10
#define RUNNING_PROGRAM 0
// #define DEBUG_MODE

Motors *m;
Gyro *g;
ControllerReadings cr;
MTF02P *mtf02p;
DronePosition p;
Estimator *e = new Estimator();
float currentTime, lastTime = 0;
uint32_t loopTimer;

void updateSensors(float dt)
{
  g->updateData(dt);
  mtf02p->update();
}

void calculatePid()
{
  // p.useZPositioning = true;
  // calculatePidPosZ(&p, cr, e);
  calculatePidThrottle(&p, cr, e);

  if (e->isPositioningAvailable) {
    calculatePidPosX(&p, cr, e);
    calculatePidPosY(&p, cr, e);
    p.usePositioning = true;
    calculatePidVelX(&p, cr, e);
    calculatePidVelY(&p, cr, e);
    p.usePositioning = true;
  }

  calculatePidPitch(&p, cr, g);
  calculatePidRoll(&p, cr, g);
  calculatePidPitchRate(&p, cr, g);
  calculatePidRollRate(&p, cr, g);
  calculatePidYaw(&p, g);

  p.usePositioning = false;
  p.useZPositioning = false;
}

void runDrone(float dt)
{
  getRemoteCommands(&cr);
#ifndef DEBUG_MODE
  if (cr.armed)
#else
  if (true)
#endif
  {
    updateSensors(dt);
    e->calculateEstimations(g, mtf02p, dt);
    calculatePid();
    // Serial.println(e->estimatedVelZ);
#ifndef DEBUG_MODE
    m->writeDronePosition(&p);
#endif
  }
  else
  {
    resetPid(&p);
    cr.desiredHeight = 0;
    e->resetEstimations();
    g->resetState();
    m->writeAll(0);
  }
  while (micros() - loopTimer < 4000)
    ;
  loopTimer = micros();
}

void setup()
{
  Serial.begin(9600);
  Serial1.begin(9600);
  Serial2.begin(115200);
  Wire.begin();

  m = MotorsSingleton::getInstance();
  g = GyroSingleton::getInstance();
  mtf02p = MTF02PSingleton::getMTF02PInstance();
  loopTimer = micros();

  if (RUNNING_PROGRAM == 0 || RUNNING_PROGRAM == 7)
  {
#ifndef DEBUG_MODE
    m->writeAll(180);
    delay(15000);
    m->writeAll(0);
    delay(10000);
#endif
    g->readCalibration();
  }
}

void loop()
{
  currentTime = micros();
  float dt = (currentTime - lastTime) / 1000000.0;

  switch (RUNNING_PROGRAM)
  {
  case 0:
    runDrone(dt);
    break;
  case 2:
    g->test();
    delay(100);
    break;
  // case 3:
  //   gy->test();
  //   // delay(10);
  //   break;
  case 5:
    g->calibrate();
    break;
  case 6:
    m->writeAll(180);
    delay(15000);
    m->writeAll(0);
    delay(99999999);
  case 7:
    m->testMotors();
    break;
  case 8:
    g->showCalibrationValues();
    break;
  // case 9:
  //   px4flow->test();
  //   delay(70);
  //   break;
  case 10:
    g->updateData(dt);
    mtf02p->update();
    float cX = mtf02p->calibrateX(g->gY);
    float cY = mtf02p->calibrateY(g->gX);
    Serial.println(String(cX) + " " + String(cY));
    break;
  }

  lastTime = currentTime;
}
