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



    public:

        Communication(SensorSystem& sensors, Control& control, Actuators& actuators, Hardware& hardware) : rf95(RFM95_CS, RFM95_INT), 
                        sensors(sensors), control(control), actuators(actuators), hardware(hardware) {}
       
                        
        bool initialize();
        bool initializeLoRa();
        String checkForCommands();
        void sendData();
        void sendDataNoDelay();
        void update();
        

    };



#endif // COMMUNICATION_H