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

// Ring buffer class
class FlightDataBuffer {
private:
    const static int BUFFER_SIZE = 2150;  // 5 seconds at 1000Hz
    FlightDataEntry buffer[BUFFER_SIZE];
    volatile int writeIndex = 0;
    volatile bool bufferFull = false;
    volatile bool loggingActive = true;

public:
    FlightDataBuffer() {
        writeIndex = 0;
        bufferFull = false;
        loggingActive = true;
    }

    // Store a data entry in the buffer
    bool storeData(const FlightDataEntry& entry) {
        if (bufferFull || !loggingActive) return false;
        
        buffer[writeIndex] = entry;
        writeIndex++;
        
        if (writeIndex >= BUFFER_SIZE) {
            bufferFull = true;
            return false;
        }
        
        return true;
    }

    // Dump data to Serial5
    void dumpToSerial() {
        loggingActive = false;
        
        Serial.println("Dumping flight log to SD card...");
        Serial5.println("FLIGHT_LOG_BEGIN");
        delay(100);
        
        // Write CSV header
        Serial5.println("timestamp,gx,gy,gz,q0,q1,q2,q3,roll,pitch,yaw,ax,ay,az,alt,press,temp,gimbal1,gimbal2,servo1,servo2,cont1,cont2,state,dt");
        
        // Write all entries
        for (int i = 0; i < writeIndex; i++) {
            FlightDataEntry& entry = buffer[i];
            
            // Timestamp
            Serial5.print(entry.timestamp);
            Serial5.print(",");
            
            // Gyro data
            for (int j = 0; j < 3; j++) {
                Serial5.print(entry.gyro[j], 6);
                Serial5.print(",");
            }
            
            // Quaternions
            for (int j = 0; j < 4; j++) {
                Serial5.print(entry.quaternions[j], 6);
                Serial5.print(",");
            }
            
            // Euler angles
            for (int j = 0; j < 3; j++) {
                Serial5.print(entry.eulerAngles[j], 6);
                Serial5.print(",");
            }
            
            // Accelerometer
            for (int j = 0; j < 3; j++) {
                Serial5.print(entry.accelerometer[j], 6);
                Serial5.print(",");
            }
            
            // Altitude, pressure, temperature
            Serial5.print(entry.altitude, 6);
            Serial5.print(",");
            Serial5.print(entry.pressure, 6);
            Serial5.print(",");
            Serial5.print(entry.temperature, 6);
            Serial5.print(",");
            
            // Servo positions
            for (int j = 0; j < 2; j++) {
                Serial5.print(entry.servoPositions[j], 6);
                Serial5.print(",");
            }

            for (int j = 0; j < 2; j++) {
                Serial5.print(entry.gimbalPositions[j], 6);
                Serial5.print(",");
            }
            
            // Continuity
            for (int j = 0; j < 2; j++) {
                Serial5.print(entry.continuity[j], 6);
                Serial5.print(",");
            }
            
            // State and dt
            Serial5.print(entry.flightState);
            Serial5.print(",");
            Serial5.println(entry.dt, 6);
            
            // Add a small delay every 50 entries to prevent buffer overrun
            if (i % 50 == 0) {
                delay(10);
            }
        }
        
        Serial5.println("FLIGHT_LOG_END");
        Serial.print("Flight log dumped: ");
        Serial.print(writeIndex);
        Serial.println(" entries");
        
        // Reset for next flight if needed
        resetBuffer();
    }

    // Reset the buffer for next flight
    void resetBuffer() {
        writeIndex = 0;
        bufferFull = false;
        loggingActive = true;
    }

    // Check if buffer is full
    bool isFull() {
        return bufferFull;
    }

    // Get number of entries stored
    int getEntryCount() {
        return writeIndex;
    }
    
    // Pause/resume logging
    void setLogging(bool enable) {
        loggingActive = enable;
    }
};

#endif // RINGBUFFER_H