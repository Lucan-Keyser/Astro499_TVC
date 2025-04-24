/** hardware.h
* ===========================================================
* Name: Flight Controller Hardware Interface
* Section: TVC499
* Project: Flight Controller
* Purpose: LED and pyro channel control
* ===========================================================
*/

#ifndef FLIGHTCOMPUTER_H
#define FLIGHTCOMPUTER_H

#include <Arduino.h>
#include <FastLED.h>
#include "../include/sensors.h"
#include "../include/communication.h"
#include "../include/hardware.h"
#include "../include/sensors.h"
#include "../include/control.h"
#include "../include/state.h"
#include "../include/logging.h"
#include "../include/config.h"
#include "../include/ringbuffer.h"


class FlightComputer {
    private:
        SensorSystem sensors;  
        Actuators actuators; // Actuator system object
        Hardware hardware; // Hardware system object
        Control control; // Control system object
        Communication communication; // Communication system object




    public:
        FlightComputer() : control(sensors, actuators), communication(sensors, control, actuators, hardware) {}
       
        bool initialize();
        void update();
        

    };



#endif // HARDWARE_H