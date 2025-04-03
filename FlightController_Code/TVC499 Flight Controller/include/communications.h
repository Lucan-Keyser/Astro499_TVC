#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include <Arduino.h>
#include <RH_RF95.h>

/** 
 * @brief initialize the LoRa communication
 * @pre 
*/ 
void initializeCommunication();

/** 
 * @brief quotient of two ints 
 * @param aNum the dividend 
 * @param bNum the divisor 
 * @return quotient of a divided by b 
*/ 
void checkForCommands();

// Process serial commands
void readSerial();

// Send telemetry data
void sendData();

// External variables to be accessed from main
extern RH_RF95 rf95;
extern unsigned long lastTelemetryTime;

#endif // COMMUNICATION_H