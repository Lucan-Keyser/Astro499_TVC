/** logging.h
* ===========================================================
* Name: Flight Controller Logging Interface
* Section: TVC499
* Project: Flight Controller
* Purpose: Data logging and telemetry
* ===========================================================
*/

#ifndef LOGDATA_H
#define LOGDATA_H

#include <Arduino.h>
#include <PWMServo.h>
#include <RH_RF95.h>
#include "../include/state.h"
#include "../include/config.h"
#include "../include/sensors.h"
#include <sensors.h>
#include "../include/control.h"
#include "../include/hardware.h"
#include "../include/communication.h"

#include "../include/config.h"
#include <SdFat.h>

#define SD_CONFIG SdioConfig(FIFO_SDIO)


class LogData {
    private:
    struct FlightDataEntry {
        unsigned long timestamp;  // milliseconds since boot
        double gyro[3];          // gyro readings
        double quaternions[4];   // orientation
        double accelerometer[3]; // acceleration values
        double altitude;         // current altitude
        double servoPositions[2]; // servo positions
        double gimbalPositions[2];
        double continuity[2];    // pyro continuity readings
        int flightState;         // current state of flight controller
        double dt;               // time delta
    };
    
        SensorSystem& sensors; // Sensor system object
        Control& control; // Control system object
        Actuators& actuators; // Actuator system object
        Hardware& hardware; // Hardware system object

        FlightDataEntry buffer[BUFFER_SIZE];
        int writeIndex = 0;
        bool bufferFull = false;
        bool loggingActive = true;

        SdFs sd;
        FsFile dataFile;

        char filename[32];

    public:
        LogData(SensorSystem& sensors, Control& control, Actuators& actuators, Hardware& hardware) : sensors(sensors),
             control(control), actuators(actuators), hardware(hardware) {}

       
        bool initialize();
        void resetBuffer();
        bool isBufferFull();
        void logFlightData(int state);
        int getEntryCount();
        void setLogging(bool enable);
        bool dumpToSD();
        bool storeData(const FlightDataEntry& entry);
  

    };




#endif // LOGDATA_H