/** communication.h
* ===========================================================
* Name: Flight Controller Communication Interface
* Section: TVC499
* Project: Flight Controller
* Purpose: LoRa radio communication and serial interface
* ===========================================================
*/

#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include <Arduino.h>
#include <PWMServo.h>
#include <RH_RF95.h>
#include "../include/config.h"
#include "../include/control.h"
#include "../include/sensors.h"
#include "../include/actuators.h"
#include "../include/hardware.h"

#define RFM95_CS 10
#define RFM95_RST 23
#define RFM95_INT 1
#define RF95_FREQ 915.0

// RF95 LoRa radio settings


class Communication {
    private:
        RH_RF95 rf95; // Pointer to RF95 LoRa radio object
        SensorSystem& sensors; // Sensor system object
        Control& control; // Control system object
        Actuators& actuators; // Actuator system object
        Hardware& hardware; // Hardware system object
        double lastTelemetryTime = millis(); // Last telemetry time
        double telemetryInterval = 1000; // Telemetry interval in milliseconds

        struct TelemetryData {
            float yaw; // Yaw angle in degrees
            float pitch; // Pitch angle in degrees
            float roll; // Roll angle in degrees
            float altitude; // Altitude in meters
            float yawServo; // Yaw servo angle in degrees
            float pitchServo; // Pitch servo angle in degrees
            float continuity1; // Continuity 1 value - 0 or 1 
            float continuity2; // Continuity 2 value - 0 or 1
            float dt; // Time delta in seconds for main loop
            float state; // Flight state - 0 to 7
            float sdLogState; // Serial SD boolean value - 0 1 or 2
        };



    public:

        Communication(SensorSystem& sensors, Control& control, Actuators& actuators, Hardware& hardware) : rf95(RFM95_CS, RFM95_INT), 
                        sensors(sensors), control(control), actuators(actuators), hardware(hardware) {}
       

        bool initialize();
        bool initializeLoRa();
        String checkForCommands();
        void sendData(int state, int sdLogState);
        void sendDataNoDelay(int state, int sdLogState);

        

    };



#endif // COMMUNICATION_H