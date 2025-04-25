#include "../include/logdata.h"
#include "../include/config.h"
#include "../include/communication.h"
#include "../include/hardware.h"

bool LogData::initialize() {
    bool success = true;
    // Initialize the flight data buffer
    resetBuffer();
    return success;

}

void LogData::logFlightData(int state) {
    // Create flight data entry
    FlightDataEntry entry;
    
    // Fill in data
    entry.timestamp = millis();

    for (int i = 0; i < 3; i++) {
        entry.gyro[i] = sensors.getGyroRates()[i];
    }

    for (int i = 0; i < 4; i++) {
        entry.quaternions[i] = sensors.getQuaternions()[i];
    }

    for (int i = 0; i < 3; i++) {
        entry.accelerometer[i] = sensors.getAccelerometer()[i];
    }

    entry.altitude = sensors.getAltitude();

    for (int i = 0; i < 2; i++) {
        entry.gimbalPositions[i] = control.getGimbalAngles()[i];
    }

    for (int i = 0; i < 2; i++) {
        entry.servoPositions[i] = actuators.getServoAngles()[i];
    }

    entry.continuity[0] = hardware.getPyroContinuity1();
    entry.continuity[1] = hardware.getPyroContinuity2();
    entry.altitude = sensors.getAltitude();
    entry.flightState = state;
    entry.dt = sensors.getDt();
    
    // Store in the buffer
    /*bool success =*/ storeData(entry);
    
    // Debug info (only log occasionally to avoid slowing down)
    static unsigned long lastDebugTime = 0;
    if (millis() - lastDebugTime > 5000) {
        Serial.print("Flight log entries: ");
        Serial.print(getEntryCount());
        Serial.print(", Buffer full: ");
        Serial.println(isBufferFull() ? "YES" : "NO");
        lastDebugTime = millis();
    }
}


void LogData::resetBuffer() {
    writeIndex = 0;
    bufferFull = false;
    loggingActive = true;
}

bool LogData::isBufferFull() {
    return bufferFull;
}

int LogData::getEntryCount() {
    return writeIndex;
}

void LogData::setLogging(bool enable) {
    loggingActive = enable;
}

bool LogData::storeData(const FlightDataEntry& entry) {
    if (bufferFull || !loggingActive) return false;
    
    buffer[writeIndex] = entry;
    writeIndex++;
    
    if (writeIndex >= BUFFER_SIZE) {
        bufferFull = true;
        return false;
    }
    
    return true;
}

bool LogData::dumpToSD() {
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