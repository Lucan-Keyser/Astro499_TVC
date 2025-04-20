/** ringbuffer.h
* ===========================================================
* Name: Flight Data Ring Buffer Interface
* Section: TVC499
* Project: Flight Controller
* Purpose: Data logging and storage for flight data
* ===========================================================
*/

#ifndef RINGBUFFER_H
#define RINGBUFFER_H

#include <Arduino.h>

// Flight data structure that will be stored in the ring buffer
struct FlightDataEntry {
    unsigned long timestamp;  // milliseconds since boot
    double gyro[3];          // gyro readings
    double quaternions[4];   // orientation
    double eulerAngles[3];   // roll, pitch, yaw
    double accelerometer[3]; // acceleration values
    double altitude;         // current altitude
    double pressure;         // pressure reading
    double temperature;      // temperature reading
    double servoPositions[2]; // servo positions
    double gimbalPositions[2];
    double continuity[2];    // pyro continuity readings
    int flightState;         // current state of flight controller
    double dt;               // time delta
};



/**
 * @brief Initialize the flight data buffer
 */
void initializeBuffer();

/**
 * @brief Store a data entry in the buffer
 * @param entry Flight data entry to store
 * @return True if successful, false if buffer full or logging inactive
 */
bool storeData(const FlightDataEntry& entry);

/**
 * @brief Dump all buffer data to Serial5
 */
void dumpToSerial();

/**
 * @brief Reset the buffer for a new flight
 */
void resetBuffer();

/**
 * @brief Check if buffer is full
 * @return True if buffer is full, false otherwise
 */
bool isBufferFull();

/**
 * @brief Get number of entries stored
 * @return Number of entries in buffer
 */
int getEntryCount();

/**
 * @brief Enable or disable logging
 * @param enable True to enable logging, false to disable
 */
void setLogging(bool enable);

#endif // RINGBUFFER_H