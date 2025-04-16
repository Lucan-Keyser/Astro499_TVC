#ifndef STATE_H
#define STATE_H

#include <Arduino.h>

void stateMachine(int& state, double* acceleration, double* eulerAngles, double* altData);

void updateState(int& state, double* acceleration, double* eulerAngles, double* altData);

bool checkAbort(double* eulerAngles);

bool detectApogee(double* altData);

#endif // SENSORS_H