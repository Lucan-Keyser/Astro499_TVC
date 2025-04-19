#include <Arduino.h>
#include <PWMServo.h>
#include <RH_RF95.h> // Include the header file for RH_RF95
#include <DMAChannel.h>  // Add this include

// Existing definitions...

// Add these prototypes
void setupSerialDMA();
void dmaCompleteCallback();
void sendSerialDMA(const uint8_t* data, size_t size);
bool isDMAActive();


#ifndef LOGGING_H
#define LOGGING_H

void printToCSV();

void sendToLog(RH_RF95* rf95);

void logGlobalData (double* gyroRates, double* quaternions, double* eulerAngles, double* accelerometer, double refPressure, double* altData, double st, double dt, double* continuity);

void logControlData (double* gimbal, double* servo);




#endif