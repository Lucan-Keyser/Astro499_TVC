#ifndef LOGGING_H
#define LOGGING_H

#include <Arduino.h>
#include <PWMServo.h>
#include <RH_RF95.h>
#include "ringbuffer.h"

// External declarations
extern FlightDataBuffer flightLog;


// Initialize logging system
void initializeLogging();

// Log flight data to ring buffer
void logFlightData();

// Log control data (for backward compatibility)
void logControlData(double* gimbal, double* servo);

void logGlobalData (double* gyroRates, double* quaternions, double* eulerAngles, double* accelerometer, double refPressure, double* altData, double st, double dt, double* continuity);

void sendToLog (RH_RF95* rf95);

void updateSerialLog (RH_RF95* rf95, double setpoint);

// Process serial commands
void checkForLogCommands();



#endif // LOGGING_H