#ifndef STATE_H
#define STATE_H

#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO08x.h>
#include <Adafruit_BMP3XX.h>
#include <utility/imumaths.h>

void stateMachine(Adafruit_BNO08x* bno, Adafruit_BMP3XX* bmp, int& state, double* accelerometer, double* eulerAngles, double* altData, double* quaternions, double& refPressure);

void updateState(Adafruit_BNO08x* bno, Adafruit_BMP3XX* bmp, int& state, double* accelerometer, double* eulerAngles, double* altData, double* quaternions, double& refPressure);

bool checkAbort(double* eulerAngles);

bool detectApogee(double* altData);

#endif // SENSORS_H