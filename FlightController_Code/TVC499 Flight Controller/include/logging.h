#include <Arduino.h>
#include <PWMServo.h>


#ifndef LOGGING_H
#define LOGGING_H

void printToCSV();

void sendToLog();

void logGlobalData (double* gyroRates, double* quaternions, double* eulerAngles, double* accelerometer, double refPressure, double* altData, double st, double dt);

void logControlData (double* gimbal, double* servo);




#endif