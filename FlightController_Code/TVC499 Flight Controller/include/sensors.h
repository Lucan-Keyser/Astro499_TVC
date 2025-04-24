// SensorSystem.h
#ifndef SENSOR_SYSTEM_H
#define SENSOR_SYSTEM_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO08x.h>
#include <Adafruit_BMP3XX.h>

class SensorSystem {
private:
    Adafruit_BNO08x bno;
    Adafruit_BMP3XX bmp;

    double gyroOffsets[3] = {0.0, 0.0, 0.0}; // Gyro offsets for calibration
    double altData[3] = {0.0, 0.0, 0.0}; // Altitude data array
    double quaternions[4] = {1.0, 0.0, 0.0, 0.0}; // Quaternion data
    double accelerometer[3] = {0.0, 0.0, 0.0}; // Accelerometer data array
    double eulerAngles[3] = {0.0, 0.0, 0.0}; // Euler angles array
    double gyroRates[3] = {0.0, 0.0, 0.0}; // Gyroscope rates array
    double lastSensorTime = micros();
    double dt = 0; // Time delta for sensor updates
    double refPressure = 1013.25; // Reference pressure in hPa;

    
public:

    bool initialize();
    void updateIMU();
    bool updateAltimeter();
    bool resetSensors();
    bool resetIMU();
    bool zeroAltimeter();
    void initializeQuaternions();

    //getter functions
    double* getGyroRates() { return gyroRates; }
    double* getQuaternions() { return quaternions; }
    double* getEulerAngles() { return eulerAngles; }
    double* getAccelerometer() { return accelerometer; }
    double* getAltData() { return altData; }
    double getAltitude() { return altData[0]; }
    double getPressure() { return altData[1]; }
    double getTemperature() { return altData[2]; }

};

#endif // SENSOR_SYSTEM_H