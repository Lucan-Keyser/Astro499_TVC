/** ringbuffer.cpp
* ===========================================================
* Name: Flight Data Ring Buffer Implementation
* Section: TVC499
* Project: Flight Controller
* Purpose: Data logging and storage implementation
* ===========================================================
*/

#include "../include/ringbuffer.h"
#include "../include/config.h"

// Global buffer and state variables
static FlightDataEntry buffer[BUFFER_SIZE];
static volatile int writeIndex = 0;
static volatile bool bufferFull = false;
static volatile bool loggingActive = true;

void initializeBuffer() {
    writeIndex = 0;
    bufferFull = false;
    loggingActive = true;
}

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
            Serial5.print(entry.gimbalPositions[j], 6);
            Serial5.print(",");
        }
        
        for (int j = 0; j < 2; j++) {
            Serial5.print(entry.servoPositions[j], 6);
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

void resetBuffer() {
    writeIndex = 0;
    bufferFull = false;
    loggingActive = true;
}

bool isBufferFull() {
    return bufferFull;
}

int getEntryCount() {
    return writeIndex;
}

void setLogging(bool enable) {
    loggingActive = enable;
}