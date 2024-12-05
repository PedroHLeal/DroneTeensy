#include <Arduino.h>
#include <Wire.h>
#include "motors.h"
#include "sensors/gyro.h"
#include "sensors/range_sensor.h"
#include "comms/controller_readings.h"
#include "comms/bluetooth.h"
#include "sensors/px4flow.h"
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
#define RUNNING_PROGRAM 0

Motors *m;
Gyro *g;
RangeSensor *range, *sr04;
ControllerReadings cr;
PX4Flow *px4flow;
DronePosition p;
Estimator *e = new Estimator();
float currentTime, lastTime = 0;
uint32_t loopTimer;

void updateSensors(float dt)
{
  g->updateData(dt);
  sr04->update();
  px4flow->update_integral();
}

void calculatePid()
{
  calculatePidThrottle(&p, cr, e);
  // calculatePidVelX(&p, cr, px4flow, sr04);
  // calculatePidVelY(&p, cr, px4flow, sr04);
  calculatePidPitch(&p, cr, g);
  calculatePidRoll(&p, cr, g);
  calculatePidPitchRate(&p, cr, g);
  calculatePidRollRate(&p, cr, g);
  calculatePidYaw(&p, g);
}

void runDrone(float dt)
{
  getRemoteCommands(&cr);
  if (cr.armed)
  {
    updateSensors(dt);
    e->calculateEstimations(g, px4flow, sr04, dt);
    calculatePid();
    // Serial.println(p.usePositioning);
    // Serial.println(String(p.throttle) + " " + String(p.pitch) + " " + String(p.yaw));
    m->writeDronePosition(p);
  }
  else
  {
    resetPid(&p);
    cr.desiredHeight = 0;
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
  Wire.begin();

  m = MotorsSingleton::getInstance();
  g = GyroSingleton::getInstance();
  range = RangeSensorSingleton::getInstance(GYUS42);
  sr04 = RangeSensorSingleton::getInstance(HC_SR04);
  px4flow = PX4FlowSingleton::getPX4FlowInstance();

  loopTimer = micros();

  if (RUNNING_PROGRAM == 0 || RUNNING_PROGRAM == 7)
  {
    m->writeAll(180);
    delay(15000);
    m->writeAll(0);
    delay(10000);
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
  case 3:
    sr04->test();
    delay(20);
    break;
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
  case 9:
    px4flow->test();
    delay(70);
    break;
  }

  lastTime = currentTime;
}
