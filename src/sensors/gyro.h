#ifndef _GYRO
#define _GYRO

class Gyro
{
public:
    void readCalibration();
    bool calibrate();
    void test();
    Gyro();
    bool updateData(float dt);
    float getVerticalAcceleration();
    void resetState();
    float
        aX = 0,
        aY = 0, aZ = 0,
        gX = 0, gY = 0, gZ = 0,
        gPosX = 0, gPosY = 0, gPosZ = 0,
        gPosXUntertainty = 0, gPosYUncertainty = 0, gPosZUncertainty = 0,
        aAngleX = 0, aAngleY = 0, aAngleZ = 0,
        inertialAccelX = 0, inertialAccelY = 0, inertialAccelZ = 0,
        posX = 0, posY = 0, verticalSpeed = 0;
    void saveCalibration(
        float gErrorX,
        float gErrorY,
        float gErrorZ,
        float aErrorX,
        float aErrorY,
        float aErrorZ);

    void loadCalibration(
        float *gErrorX,
        float *gErrorY,
        float *gErrorZ,
        float *aErrorX,
        float *aErrorY,
        float *aErrorZ);
    void showCalibrationValues();

private:
    int calibrationRounds = 3000, currentRound = 0;
    int samplesToIgnore = 3000, ignoredSamples = 0;
    bool shouldCalibrate = false;
    int calibrationIndication = 0;
    float calibrationLedTime = 0;
    float
        rawAX = 0,
        rawAY = 0, rawAZ = 0,
        rawGX = 0, rawGY = 0, rawGZ = 0,
        gErrorX = 0, gErrorY = 0, gErrorZ = 0,
        aErrorX = 0, aErrorY = 0, aErrorZ = 0,
        tmp = 0;
    void setAngle(float dt);
    void readRawValues();
};

class GyroSingleton
{
public:
    static Gyro *getInstance()
    {
        if (g == nullptr)
        {
            g = new Gyro();
        }
        return g;
    }

private:
    static Gyro *g;
};

#endif