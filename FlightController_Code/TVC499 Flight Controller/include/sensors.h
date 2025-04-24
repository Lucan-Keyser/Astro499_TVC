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
    
public:
    
    bool initialize(double* quaternions, double* accelerometer);
    void updateIMU(double* gyroRates, double* quaternions, double* eulerAngles, double* accelerometer, double dt);
    bool updateAltimeter(double* altData);
    void resetSensors(double* quaternions, double* accelerometer);
    void zeroAltimeter();
    void initializeQuaternions(double* quaternions, double* accelerometer);
};

#endif // SENSOR_SYSTEM_H