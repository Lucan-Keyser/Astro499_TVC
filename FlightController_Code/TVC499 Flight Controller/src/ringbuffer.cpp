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
#include <SdFat.h>

#define SD_CONFIG SdioConfig(FIFO_SDIO)

// Global buffer and state variables
static FlightDataEntry buffer[BUFFER_SIZE];
static volatile int writeIndex = 0;
static volatile bool bufferFull = false;
static volatile bool loggingActive = true;

// SD card objects
SdFs sd;
FsFile dataFile;
char filename[32];

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

bool dumpToSD() {
    SdFs sd;
    FsFile dataFile;
    char filename[32];
    
    // Initialize SD card
    if (!sd.begin(SdioConfig(FIFO_SDIO))) {
        Serial.println("SD initialization failed!");
        return false;
    }
    
    sprintf(filename, "FLIGHT_%lu.CSV", millis());
    if (!dataFile.open(filename, O_WRITE | O_CREAT)) {
        Serial.println("Failed to create file!");
        return false;
    }
    
    // Write CSV header
    dataFile.println("timestamp,gx,gy,gz,q0,q1,q2,q3,ax,ay,az,alt,gimbal1,gimbal2,servo1,servo2,cont1,cont2,state,dt");
    
    // Use a small 512-byte static buffer (one SD card block)
    char lineBuffer[512];
    
    // Process all entries
    for (int i = 0; i < writeIndex; i++) {
        // Format data line by line
        int len = snprintf(lineBuffer, sizeof(lineBuffer),
            "%lu,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%d,%.6f\n",
            buffer[i].timestamp,
            buffer[i].gyro[0], buffer[i].gyro[1], buffer[i].gyro[2],
            buffer[i].quaternions[0], buffer[i].quaternions[1], buffer[i].quaternions[2], buffer[i].quaternions[3],
            buffer[i].accelerometer[0], buffer[i].accelerometer[1], buffer[i].accelerometer[2],
            buffer[i].altitude, 
            buffer[i].gimbalPositions[0], buffer[i].gimbalPositions[1],
            buffer[i].servoPositions[0], buffer[i].servoPositions[1],
            buffer[i].continuity[0], buffer[i].continuity[1],
            buffer[i].flightState, buffer[i].dt);
            
        dataFile.write(lineBuffer, len);
    }
    
    dataFile.flush();
    dataFile.close();
    
    Serial.println("Data successfully written to SD");
    return true;
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