/** logging.h
* ===========================================================
* Name: Flight Controller Logging Interface
* Section: TVC499
* Project: Flight Controller
* Purpose: Data logging and telemetry
* ===========================================================
*/

#ifndef LOGGING_H
#define LOGGING_H

#include <Arduino.h>
#include <PWMServo.h>
#include <RH_RF95.h>
#include "../include/ringbuffer.h"

/**
 * @brief Initialize logging system
 */
void initializeLogging();

/**
 * @brief Log flight data to ring buffer
 */
void logFlightData();

/**
 * @brief Log control data
 * @param gimbal Gimbal positions array
 * @param servo Servo positions array
 */
void logControlData(double* gimbal, double* servo);

/**
 * @brief Log global flight data
 * @param gyroRates Gyroscope rates array
 * @param quaternions Quaternion values array
 * @param eulerAngles Euler angles array
 * @param accelerometer Accelerometer values array
 * @param refPressure Reference pressure
 * @param altData Altitude data array
 * @param st Flight state
 * @param dt Time delta
 * @param continuity Continuity values array
 */
void logGlobalData(double* gyroRates, double* quaternions, double* eulerAngles, 
                  double* accelerometer, double refPressure, double* altData, 
                  double st, double dt, double* continuity);

/**
 * @brief Send logged data to telemetry
 * @param rf95 LoRa radio object pointer
 */
void sendToLog(RH_RF95* rf95);

/**
 * @brief Update serial log with setpoint
 * @param rf95 LoRa radio object pointer
 * @param setpoint Serial boolean setpoint
 */
void updateSerialLog(RH_RF95* rf95, double setpoint);

/**
 * @brief Process serial commands for logging
 */
void checkForLogCommands();

#endif // LOGGING_H