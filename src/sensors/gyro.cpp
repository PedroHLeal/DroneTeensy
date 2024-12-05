#include <Wire.h>
#include <EEPROM.h>
#include "gyro.h"
#include "../filters.h"

Gyro *GyroSingleton::g = nullptr;
const int MPU = 0x68;

Gyro::Gyro()
{
    Wire.beginTransmission(MPU);
    Wire.write(0x6B);
    Wire.write(0);
    Wire.endTransmission(true);
}

void Gyro::readRawValues()
{
    Wire.beginTransmission(0x68);
    Wire.write(0x1A);
    Wire.write(0x05);
    Wire.endTransmission();
    Wire.beginTransmission(0x68);
    Wire.write(0x1C);
    Wire.write(0x10);
    Wire.endTransmission();
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 14);
    
    delay(1);

    rawAX = float(int16_t(Wire.read() << 8 | Wire.read())) / 4096.0; // accX
    rawAY = float(int16_t(Wire.read() << 8 | Wire.read())) / 4096.0; // accY
    rawAZ = float(int16_t(Wire.read() << 8 | Wire.read())) / 4096.0; // accZ
    tmp = float(Wire.read() << 8 | Wire.read());                     // tmp
    rawGX = float(int16_t(Wire.read() << 8 | Wire.read())) / 131.0;  // gyroX
    rawGY = float(int16_t(Wire.read() << 8 | Wire.read())) / 131.0;  // gyroY
    rawGZ = float(int16_t(Wire.read() << 8 | Wire.read())) / 131.0;  // gyroZ
}

bool Gyro::updateData(float dt)
{
    this->readRawValues();

    aX = rawAX - this->aErrorX;
    aY = rawAY - this->aErrorY;
    aZ = rawAZ;
    gX = rawGX - this->gErrorX;
    gY = rawGY - this->gErrorY;
    gZ = rawGZ - this->gErrorZ;

    this->setAngle(dt);

    return true;
}

bool Gyro::calibrate()
{
    this->readRawValues();
    if (this->currentRound < this->calibrationRounds)
    {
        aErrorX += rawAX;
        aErrorY += rawAY;
        aErrorZ += rawAZ;
        gErrorX += rawGX;
        gErrorY += rawGY;
        gErrorZ += rawGZ;

        this->currentRound++;
        return false;
    }
    else
    {
        gErrorX /= calibrationRounds;
        gErrorY /= calibrationRounds;
        gErrorZ /= calibrationRounds;

        aErrorX /= calibrationRounds;
        aErrorY /= calibrationRounds;
        aErrorZ /= calibrationRounds;

        saveCalibration(gErrorX, gErrorY, gErrorZ, aErrorX, aErrorY, aErrorZ);
        return true;
    }
}

void Gyro::readCalibration()
{
    loadCalibration(&gErrorX, &gErrorY, &gErrorZ, &aErrorX, &aErrorY, &aErrorZ);
}

void Gyro::setAngle(float dt)
{
    // Detect accel angle
    aAngleX = (atan(aY / sqrt(sq(aX) + sq(aZ)))) * RAD_TO_DEG; // ANGLEROLL
    aAngleY = (-atan(aX / sqrt(sq(aY) + sq(aZ)))) * RAD_TO_DEG; // ANGLEPITCH
    aAngleZ = (atan2(aY, aZ)) * RAD_TO_DEG;

    // kalman1d(gPosX, gPosXUntertainty, gX, aAngleX, &gPosX, &gPosXUntertainty, dt);
    // kalman1d(gPosY, gPosYUncertainty, gY, aAngleY, &gPosY, &gPosYUncertainty, dt);
    gPosX = 0.98 * (gPosX + gX * dt) + 0.02 * aAngleX;
    gPosY = 0.98 * (gPosY + gY * dt) + 0.02 * aAngleY;

    gPosZ += gZ * dt;

    inertialAccelX = aX * cos(gPosY * DEG_TO_RAD) + aY * sin(gPosX  * DEG_TO_RAD) * sin(gPosY * DEG_TO_RAD) + aZ * sin(gPosY * DEG_TO_RAD) * cos(gPosX * DEG_TO_RAD);
    inertialAccelY = aX * sin(gPosX * DEG_TO_RAD) * sin(gPosY * DEG_TO_RAD) + aY * cos(gPosX * DEG_TO_RAD) - aZ * sin(gPosX * DEG_TO_RAD) * cos(gPosY * DEG_TO_RAD);
    inertialAccelZ = -aX * sin(gPosY * DEG_TO_RAD) + aY * cos(gPosY * DEG_TO_RAD) * sin(gPosX * DEG_TO_RAD) + aZ * cos(gPosY * DEG_TO_RAD) * cos(gPosX * DEG_TO_RAD);

    // Serial.println(String(aX) + " " + String(aY) + " " + String(gX) + " " + String(gY));
    // Serial.println(String(aAngleX) + " " + String(aAngleY));
    // Serial.println(String(gPosX) + " " + String(gPosY));
}

void Gyro::resetState()
{
    gPosX = 0;
    gPosY = 0;
    gPosZ = 0;
}

void Gyro::test()
{
    updateData(0);
    Serial.println("Gyro: " + String(gX) + " " + String(gY) + " " + String(gZ));
    Serial.println("Accel: " + String(aX) + " " + String(aY) + " " + String(aZ));
}

void Gyro::saveCalibration(
    float gErrorX,
    float gErrorY,
    float gErrorZ,
    float aErrorX,
    float aErrorY,
    float aErrorZ)
{
    Serial.println("Salvando calibragem...");
    Serial.println("Gyro: " + String(gErrorX) + " " + String(gErrorY) + " " + String(gErrorZ));
    Serial.println("Accel: " + String(aErrorX) + " " + String(aErrorY) + " " + String(aErrorZ));

    Serial.println("Salvar?");
    while (!(Serial.available() > 0))
        ;

    EEPROM.put(0, gErrorX);
    EEPROM.put(1 * sizeof(float), gErrorY);
    EEPROM.put(2 * sizeof(float), gErrorZ);
    EEPROM.put(3 * sizeof(float), aErrorX);
    EEPROM.put(4 * sizeof(float), aErrorY);
    EEPROM.put(5 * sizeof(float), aErrorZ);

    delay(99999999);
}

void Gyro::loadCalibration(
    float *gErrorX,
    float *gErrorY,
    float *gErrorZ,
    float *aErrorX,
    float *aErrorY,
    float *aErrorZ)
{
    EEPROM.get(0, *gErrorX);
    EEPROM.get(1 * sizeof(float), *gErrorY);
    EEPROM.get(2 * sizeof(float), *gErrorZ);
    EEPROM.get(3 * sizeof(float), *aErrorX);
    EEPROM.get(4 * sizeof(float), *aErrorY);
    EEPROM.get(5 * sizeof(float), *aErrorZ);
}

void Gyro::showCalibrationValues()
{
    float gErrorX = 0;
    float gErrorY = 0;
    float gErrorZ = 0;
    float aErrorX = 0;
    float aErrorY = 0;
    float aErrorZ = 0;

    EEPROM.get(0, gErrorX);
    EEPROM.get(1 * sizeof(float), gErrorY);
    EEPROM.get(2 * sizeof(float), gErrorZ);
    EEPROM.get(3 * sizeof(float), aErrorX);
    EEPROM.get(4 * sizeof(float), aErrorY);
    EEPROM.get(5 * sizeof(float), aErrorZ);

    Serial.println(String(gErrorX) + " " + String(gErrorY) + " " + String(gErrorZ) + " " + String(aErrorX) + " " + String(aErrorY) + " " + String(aErrorZ));
}